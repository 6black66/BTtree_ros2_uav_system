#include <observe/perception.hpp>
#include <uav_shared_video/frame_ring.hpp>
#include <uav_ros2_support/xml_config.hpp>
#include <msgs/msg/video_frame_ref.hpp>
#include <msgs/msg/observation.hpp>
#include <msgs/msg/observe_track_request.hpp>
#include <msgs/action/task.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <opencv2/imgproc.hpp>
#include <jsoncpp/json/json.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <cmath>
#include <random>
#include <sstream>
#include <thread>

namespace {
using Clock=std::chrono::steady_clock;
using Result=msgs::msg::Observation;
using Request=msgs::msg::ObserveTrackRequest;
using Task=msgs::action::Task;
using Goal=rclcpp_action::ServerGoalHandle<Task>;
struct Image {
  msgs::msg::VideoFrameRef ref;
  cv::Mat bgr;
  Clock::time_point received;
};
struct Snapshot {
  std::shared_ptr<Image> image;
  Result result;
};
struct Pending { Request request; Snapshot snapshot; msgs::msg::ObjectDetection target; };
struct Action { std::shared_ptr<Goal> goal; Clock::time_point deadline; Json::Value input; };

class ObserveNode:public rclcpp::Node {
 public:
  ObserveNode():Node("observe_executor") {
    using uav_ros2_support::param;
    uav_ros2_support::load_xml_overrides(*this,"observe");
    frame_age_=param<double>(*this,"max_frame_age_ms",1000.);
    seed_age_=param<double>(*this,"max_seed_age_ms",1500.);
    slow_period_=param<int>(*this,"slow_period_ms",500);
    fast_period_=param<int>(*this,"fast_period_ms",33);
    cache_limit_=param<int>(*this,"snapshot_cache_size",4);
    action_timeout_=param<int>(*this,"action_timeout_ms",4000);
    assoc_iou_=param<double>(*this,"association_iou",0.3);
    assoc_ttl_=param<double>(*this,"association_ttl_ms",2000.);
    score_=param<double>(*this,"confidence_threshold",0.45);
    nms_=param<double>(*this,"nms_threshold",0.45);
    max_detections_=param<int>(*this,"max_detections",100);
    const int threads=param<int>(*this,"opencv_threads",2);
    if(!std::isfinite(frame_age_) || !std::isfinite(seed_age_) || !std::isfinite(assoc_ttl_) ||
       !std::isfinite(assoc_iou_) || !std::isfinite(score_) || !std::isfinite(nms_) ||
       frame_age_<1 || seed_age_<1 || slow_period_<1 || fast_period_<1 || cache_limit_<1 || cache_limit_>32 ||
       action_timeout_<1 || assoc_ttl_<1 || assoc_iou_<=0 || assoc_iou_>1 || score_<=0 || score_>=1 ||
       nms_<=0 || nms_>=1 || max_detections_<1 || max_detections_>1000 || threads<1 || threads>64)
      throw std::runtime_error("invalid observe timing, cache, thread or detector parameters");
    cv::setNumThreads(threads);
    auto model=param<std::string>(*this,"model_path","models/object_detection_yolox_2022nov.onnx");
    auto labels=param<std::string>(*this,"labels_path","models/coco.names");
    // Resolve identically for standalone XML and system.launch typed overrides.
    auto dir=param<std::string>(*this,"model_dir","");
    auto file=get_parameter("config_file").as_string();
    if(dir.empty() && !file.empty()) dir=std::filesystem::absolute(file).parent_path();
    auto resolve=[&](const std::string& p) {
      return std::filesystem::path(p).is_absolute()?p:(std::filesystem::path(dir)/p).string();
    };
    model_=resolve(model);labels_=resolve(labels);
    std::random_device random;
    observation_id_=(uint64_t(random())<<32)|random();
    slow_pub_=create_publisher<Result>(param<std::string>(*this,"slow_topic","observe/slow/detections"),10);
    fast_pub_=create_publisher<Result>(param<std::string>(*this,"fast_topic","observe/fast/result"),10);
    frame_sub_=create_subscription<msgs::msg::VideoFrameRef>(param<std::string>(*this,"frame_topic","scene/frame"),
      rclcpp::QoS(1).best_effort(),[this](msgs::msg::VideoFrameRef::ConstSharedPtr msg){receive(*msg);});
    request_sub_=create_subscription<Request>(param<std::string>(*this,"request_topic","observe/fast/request"),
      10,[this](Request::ConstSharedPtr msg){request(*msg);});
    action_server_=rclcpp_action::create_server<Task>(this,"behaviors/observe",
      [this](const auto&,const std::shared_ptr<const Task::Goal> goal) {
        if(actions_.size()>=8 || (!goal->behavior.empty() && goal->behavior!="observe"))
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },[](const std::shared_ptr<Goal>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Goal> goal) {
        Json::Value input;Json::CharReaderBuilder builder;std::string error;
        std::istringstream text(goal->get_goal()->goal_json.empty()?"{}":goal->get_goal()->goal_json);
        if(!Json::parseFromStream(builder,text,&input,&error) || !input.isObject() ||
           (input.isMember("target_id") && !input["target_id"].isUInt64()) ||
           (input.isMember("session_id") && !input["session_id"].isUInt64()) ||
           (input.isMember("observation_id") && !input["observation_id"].isUInt64())) {
          auto result=std::make_shared<Task::Result>();result->ok=false;result->result_json=R"({"error":"invalid goal JSON or target_id"})";
          goal->abort(result);return;
        }
        actions_.push_back({goal,Clock::now()+std::chrono::milliseconds(action_timeout_),input});
      });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{complete_actions();});
    slow_thread_=std::thread([this]{slow_loop();});
    fast_thread_=std::thread([this]{fast_loop();});
    RCLCPP_INFO(get_logger(),"observe: YOLOX-S slow detector + requested KCF fast tracker; model=%s",model_.c_str());
  }
  ~ObserveNode() override {
    stopping_=true;cv_.notify_all();
    if(slow_thread_.joinable()) slow_thread_.join();
    if(fast_thread_.joinable()) fast_thread_.join();
  }
 private:
  bool fresh(const std::shared_ptr<Image>& image,double max_age) const {
    if(!image) return false;
    if(std::chrono::duration<double,std::milli>(Clock::now()-image->received).count()>max_age) return false;
    // ROS timestamps may use simulation time; compare with this node's ROS clock.
    double age=(now()-rclcpp::Time(image->ref.header.stamp,get_clock()->get_clock_type())).seconds()*1000;
    return age>=-100. && age<=max_age;
  }
  Result result_for(const std::shared_ptr<Image>& image,const std::string& status,const std::string& detail="") {
    Result result;result.status=status;result.detail=detail;
    if(image) {
      result.header=image->ref.header;result.session_id=image->ref.session_id;
      result.frame_sequence=image->ref.sequence;result.image_width=image->bgr.cols;result.image_height=image->bgr.rows;
    }
    return result;
  }
  void receive(const msgs::msg::VideoFrameRef& msg) {
    try {
      if(!reader_ || reader_name_!=msg.shm_name) {
        reader_=std::make_unique<uav::video::FrameReader>(msg.shm_name);reader_name_=msg.shm_name;
      }
      uav::video::FrameReference ref;ref.shm_name=msg.shm_name;ref.session_id=msg.session_id;
      ref.slot=msg.slot;ref.metadata.sequence=msg.sequence;
      uav::video::Frame frame;
      if(!reader_->read(ref,frame)) return;
      const auto& m=frame.metadata;
      const auto channels=uav::video::channels(m.format);
      if(!m.width || !m.height || m.width>16384 || m.height>16384 || m.step<uint64_t(m.width)*channels ||
         uint64_t(m.step)*m.height!=frame.pixels.size() || msg.width!=m.width || msg.height!=m.height ||
         msg.step!=m.step || msg.data_size!=m.data_size || msg.encoding!=uav::video::encoding(m.format) ||
         rclcpp::Time(msg.header.stamp).nanoseconds()!=m.received_stamp_ns)
        throw std::runtime_error("inconsistent shared-memory frame metadata");
      cv::Mat view(m.height,m.width,CV_MAKETYPE(CV_8U,channels),frame.pixels.data(),m.step);
      auto image=std::make_shared<Image>();image->ref=msg;image->received=Clock::now();
      if(m.format==uav::video::PixelFormat::rgb8) cv::cvtColor(view,image->bgr,cv::COLOR_RGB2BGR);
      else if(m.format==uav::video::PixelFormat::mono8) cv::cvtColor(view,image->bgr,cv::COLOR_GRAY2BGR);
      else image->bgr=view.clone();
      if(!fresh(image,frame_age_)) return;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(latest_ && latest_->ref.session_id==msg.session_id && msg.sequence<=latest_->ref.sequence) return;
        if(latest_ && (latest_->ref.session_id!=msg.session_id || latest_->bgr.size()!=image->bgr.size())) snapshots_.clear();
        latest_=std::move(image);
      }
      cv_.notify_all();
    } catch(const std::exception& e) {
      reader_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"scene frame skipped: %s",e.what());
    }
  }
  void request(const Request& req) {
    auto reject=[&](const std::string& detail) {
      Result result;result.status="rejected";result.detail=detail;result.request_id=req.request_id;
      result.session_id=req.session_id;result.observation_id=req.observation_id;fast_pub_->publish(result);
    };
    if(req.request_id.empty() || req.request_id.size()>128) {reject("request_id must contain 1..128 bytes");return;}
    std::unique_lock<std::mutex> lock(mutex_);
    // One pending operation: never silently drop a control request.
    if(pending_) {reject("another request is pending; retry after its result");return;}
    if(req.command=="stop") pending_=Pending{req,Snapshot(),msgs::msg::ObjectDetection()};
    else if(req.command=="start") {
      auto snapshot=std::find_if(snapshots_.begin(),snapshots_.end(),[&](const auto& s) {
        return s.result.session_id==req.session_id && s.result.observation_id==req.observation_id;
      });
      if(snapshot==snapshots_.end()) {reject("slow snapshot expired or unknown");return;}
      if(!fresh(snapshot->image,seed_age_) || !fresh(latest_,frame_age_) || latest_->ref.session_id!=req.session_id ||
         latest_->bgr.size()!=snapshot->image->bgr.size()) {reject("seed/frame stale or stream changed");return;}
      auto target=std::find_if(snapshot->result.detections.begin(),snapshot->result.detections.end(),[&](const auto& d){return d.target_id==req.target_id;});
      if(target==snapshot->result.detections.end() || target->width<4 || target->height<4) {reject("target ID absent or too small");return;}
      pending_=Pending{req,*snapshot,*target};
    } else {reject("command must be start or stop");return;}
    lock.unlock();cv_.notify_all();
  }
  void slow_loop() {
    std::unique_ptr<observe::Detector> detector;
    std::string model_error;
    try {detector=std::make_unique<observe::Detector>(model_,labels_,score_,nms_,max_detections_);}
    catch(const std::exception& e) {model_error=e.what();RCLCPP_ERROR(get_logger(),"detector not ready: %s",e.what());}
    observe::IdAssigner ids;
    uint64_t last_session=0,last_sequence=0;
    cv::Size last_size;
    while(!stopping_) {
      auto start=Clock::now();
      std::shared_ptr<Image> image;
      {std::lock_guard<std::mutex> lock(mutex_);image=latest_;}
      if(!detector) slow_pub_->publish(result_for(image,"error",model_error));
      else if(!fresh(image,frame_age_)) slow_pub_->publish(result_for(image,image?"stale":"no_frame","awaiting fresh scene frame"));
      else if(image->ref.session_id!=last_session || image->ref.sequence!=last_sequence) {
        try {
          if(last_session!=image->ref.session_id || last_size!=image->bgr.size()) ids.reset();
          auto detections=detector->detect(image->bgr);
          auto result=result_for(image,"ok");result.observation_id=++observation_id_;
          ids.assign(detections,std::chrono::duration<double>(image->received.time_since_epoch()).count(),assoc_iou_,assoc_ttl_/1000.);
          for(const auto& d:detections) {
            msgs::msg::ObjectDetection object;object.target_id=d.id;object.class_id=d.class_id;
            object.label=detector->label(d.class_id);object.confidence=d.confidence;
            object.x=d.box.x;object.y=d.box.y;object.width=d.box.width;object.height=d.box.height;
            result.detections.push_back(object);
          }
          result.processing_ms=std::chrono::duration<float,std::milli>(Clock::now()-start).count();
          std::lock_guard<std::mutex> lock(mutex_);
          if(latest_ && latest_->ref.session_id==image->ref.session_id && latest_->bgr.size()==image->bgr.size()) {
            if(!fresh(image,seed_age_)) {result.status="stale";result.detail="inference exceeded seed age; targets unavailable";result.detections.clear();}
            else {
              snapshots_.push_back({image,result});
              while(int(snapshots_.size())>cache_limit_) snapshots_.pop_front();
            }
            slow_pub_->publish(result);
          }
          last_session=image->ref.session_id;last_sequence=image->ref.sequence;last_size=image->bgr.size();
        } catch(const std::exception& e) {slow_pub_->publish(result_for(image,"error",e.what()));}
      }
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_until(lock,start+std::chrono::milliseconds(slow_period_),[this]{return stopping_.load();});
    }
  }
  void fast_loop() {
    observe::FastTracker tracker;
    std::optional<Pending> active;
    uint64_t sequence=0;
    auto last_update=Clock::now();
    Result idle;idle.status="idle";fast_pub_->publish(idle);
    auto emit=[&](const std::shared_ptr<Image>& image,const std::string& status,const std::string& detail,bool with_box,float ms=0) {
      auto result=result_for(image && image->ref.session_id==active->request.session_id ? image : nullptr,status,detail);
      result.session_id=active->request.session_id;result.request_id=active->request.request_id;
      result.observation_id=active->request.observation_id;result.processing_ms=ms;
      if(with_box) {auto target=active->target;target.confidence=-1;result.detections.push_back(target);}
      fast_pub_->publish(result);
    };
    while(!stopping_) {
      std::shared_ptr<Image> image;
      std::optional<Pending> pending;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock,std::chrono::milliseconds(fast_period_),[&] {
          return stopping_ || pending_ || (active && latest_ &&
            (latest_->ref.session_id!=active->request.session_id || latest_->ref.sequence>sequence) &&
            Clock::now()-last_update>=std::chrono::milliseconds(fast_period_));
        });
        if(stopping_) break;
        image=latest_;pending.swap(pending_);
      }
      if(pending) {
        if(pending->request.command=="stop") {
          Result result=result_for(image,"stopped");result.request_id=pending->request.request_id;
          fast_pub_->publish(result);active.reset();continue;
        }
        try {
          if(!fresh(pending->snapshot.image,seed_age_) || !fresh(image,frame_age_) || image->ref.session_id!=pending->request.session_id ||
             image->bgr.size()!=pending->snapshot.image->bgr.size()) throw std::runtime_error("seed/frame expired before initialization");
          observe::FastTracker candidate;
          candidate.init(pending->snapshot.image->bgr,cv::Rect2f(pending->target.x,pending->target.y,pending->target.width,pending->target.height));
          if(!fresh(pending->snapshot.image,seed_age_)) throw std::runtime_error("seed expired during KCF initialization");
          tracker=std::move(candidate);active=std::move(pending);sequence=active->snapshot.image->ref.sequence;
          emit(active->snapshot.image,"initialized","selected slow-frame target",true);
          // Release full seed image after initialization; keep only identity.
          active->snapshot=Snapshot();
        } catch(const std::exception& e) {
          Result result=result_for(image,"rejected",e.what());result.request_id=pending->request.request_id;
          result.session_id=pending->request.session_id;result.observation_id=pending->request.observation_id;fast_pub_->publish(result);
        }
      }
      if(!active) continue;
      if(!fresh(image,frame_age_) || image->ref.session_id!=active->request.session_id) {
        emit(image,"lost","frame stale or source session changed",false);active.reset();continue;
      }
      if(image->ref.sequence<=sequence) continue;
      const auto start=Clock::now();last_update=start;
      try {
        cv::Rect2f box;
        if(!tracker.update(image->bgr,box)) throw std::runtime_error("KCF lost target or image size changed; select again from slow detections");
        active->target.x=box.x;active->target.y=box.y;active->target.width=box.width;active->target.height=box.height;
        sequence=image->ref.sequence;
        // Never publish a successful tracking result after its source frame expires.
        if(!fresh(image,frame_age_)) throw std::runtime_error("tracking frame expired during processing");
        emit(image,"tracking","",true,std::chrono::duration<float,std::milli>(Clock::now()-start).count());
      } catch(const std::exception& e) {emit(image,"lost",e.what(),false);active.reset();}
    }
  }
  void complete_actions() {
    for(auto it=actions_.begin();it!=actions_.end();) {
      auto result=std::make_shared<Task::Result>();
      if(it->goal->is_canceling()) {
        result->result_json=R"({"cancelled":true})";it->goal->canceled(result);it=actions_.erase(it);continue;
      }
      std::optional<Snapshot> snapshot;
      {std::lock_guard<std::mutex> lock(mutex_);
        if(fresh(latest_,frame_age_)) for(auto candidate=snapshots_.rbegin();candidate!=snapshots_.rend();++candidate) {
          if(!fresh(candidate->image,seed_age_) || candidate->result.session_id!=latest_->ref.session_id) continue;
          if(it->input.isMember("session_id") && candidate->result.session_id!=it->input["session_id"].asUInt64()) continue;
          if(it->input.isMember("observation_id") && candidate->result.observation_id!=it->input["observation_id"].asUInt64()) continue;
          snapshot=*candidate;break;
        }}
      const msgs::msg::ObjectDetection* selected=nullptr;
      if(snapshot) for(const auto& d:snapshot->result.detections) {
        if(it->input.isMember("target_id") && d.target_id!=it->input["target_id"].asUInt64()) continue;
        if(!selected || d.confidence>selected->confidence) selected=&d;
      }
      if(selected) {
        Json::Value value;value["ok"]=true;value["target_id"]=Json::UInt64(selected->target_id);
        value["session_id"]=Json::UInt64(snapshot->result.session_id);value["observation_id"]=Json::UInt64(snapshot->result.observation_id);
        value["confidence"]=selected->confidence;value["target_type"]=selected->label;
        // Preserve legacy normalized [x,y,w,h] action JSON; typed topics use pixels.
        for(float v:{selected->x/snapshot->result.image_width,selected->y/snapshot->result.image_height,
                     selected->width/snapshot->result.image_width,selected->height/snapshot->result.image_height}) value["bbox"].append(v);
        result->ok=true;result->result_json=Json::writeString(Json::StreamWriterBuilder(),value);
        it->goal->succeed(result);it=actions_.erase(it);
      } else if(Clock::now()>=it->deadline) {
        result->result_json=R"({"ok":false,"error":"no fresh matching detection before timeout"})";
        it->goal->abort(result);it=actions_.erase(it);
      } else ++it;
    }
  }
  double frame_age_,seed_age_,assoc_iou_,assoc_ttl_,score_,nms_;
  int slow_period_,fast_period_,cache_limit_,action_timeout_,max_detections_;
  std::string model_,labels_,reader_name_;
  uint64_t observation_id_;
  std::unique_ptr<uav::video::FrameReader> reader_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> stopping_{false};
  std::shared_ptr<Image> latest_;
  std::deque<Snapshot> snapshots_;
  std::optional<Pending> pending_;
  std::vector<Action> actions_;
  std::thread slow_thread_,fast_thread_;
  rclcpp::Publisher<Result>::SharedPtr slow_pub_,fast_pub_;
  rclcpp::Subscription<msgs::msg::VideoFrameRef>::SharedPtr frame_sub_;
  rclcpp::Subscription<Request>::SharedPtr request_sub_;
  rclcpp_action::Server<Task>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<ObserveNode>());}
  catch(const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("observe"),"%s",e.what());if(rclcpp::ok()) rclcpp::shutdown();return 1;}
  if(rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
