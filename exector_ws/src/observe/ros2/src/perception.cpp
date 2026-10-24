#include <observe/perception.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>

namespace observe {
float iou(const cv::Rect2f& a,const cv::Rect2f& b) {
  const float area=(a&b).area();
  const float total=a.area()+b.area()-area;
  return total>0 ? area/total : 0;
}
void IdAssigner::assign(std::vector<Detection>& detections,double stamp,float threshold,double ttl) {
  if(stamp<=previous_stamp_ || stamp-previous_stamp_>ttl) previous_.clear();
  std::vector<std::tuple<float,size_t,size_t>> matches;
  for(size_t i=0;i<detections.size();++i) for(size_t j=0;j<previous_.size();++j) {
    float score=iou(detections[i].box,previous_[j].box);
    if(detections[i].class_id==previous_[j].class_id && score>=threshold) matches.emplace_back(score,i,j);
  }
  std::sort(matches.rbegin(),matches.rend());
  std::vector<bool> used(previous_.size(),false);
  for(const auto& match:matches) {
    auto i=std::get<1>(match),j=std::get<2>(match);
    if(!detections[i].id && !used[j]) { detections[i].id=previous_[j].id; used[j]=true; }
  }
  for(auto& d:detections) if(!d.id) d.id=next_++;
  previous_=detections; previous_stamp_=stamp;
}
Detector::Detector(const std::string& model,const std::string& labels,float confidence,float nms,int max_results)
 :confidence_(confidence),nms_(nms),max_results_(max_results) {
  std::ifstream file(labels);
  std::string line;
  while(std::getline(file,line)) { if(!line.empty() && line.back()=='\r') line.pop_back(); if(!line.empty()) labels_.push_back(line); }
  if(labels_.size()!=80) throw std::runtime_error("YOLOX-S requires exactly 80 COCO labels: "+labels);
  net_=cv::dnn::readNetFromONNX(model);
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}
std::vector<Detection> Detector::detect(const cv::Mat& bgr) {
  if(bgr.empty() || bgr.type()!=CV_8UC3) throw std::runtime_error("detector requires nonempty bgr8 image");
  float scale=std::min(640.f/bgr.cols,640.f/bgr.rows);
  cv::Mat padded(640,640,CV_8UC3,cv::Scalar(114,114,114)), resized;
  cv::resize(bgr,resized,cv::Size(std::max(1,int(bgr.cols*scale)),std::max(1,int(bgr.rows*scale))));
  resized.copyTo(padded(cv::Rect(0,0,resized.cols,resized.rows)));
  net_.setInput(cv::dnn::blobFromImage(padded,1.0,cv::Size(),cv::Scalar(),true,false));
  return decode(net_.forward(),bgr.size());
}
std::vector<Detection> Detector::decode(const cv::Mat& output,cv::Size original) const {
  if(output.dims!=3 || output.size[0]!=1 || output.size[1]!=8400 || output.size[2]!=85 || output.type()!=CV_32F || !output.isContinuous())
    throw std::runtime_error("unsupported model output: expected YOLOX-S [1,8400,85] float32");
  const float scale=std::min(640.f/original.width,640.f/original.height);
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  int row=0;
  for(int stride:{8,16,32}) for(int gy=0;gy<640/stride;++gy) for(int gx=0;gx<640/stride;++gx,++row) {
    const float* p=output.ptr<float>()+row*85;
    int cls=int(std::max_element(p+5,p+85)-(p+5));
    float score=p[4]*p[5+cls];
    if(!std::isfinite(score) || score<confidence_) continue;
    float cx=(p[0]+gx)*stride/scale,cy=(p[1]+gy)*stride/scale;
    float w=std::exp(p[2])*stride/scale,h=std::exp(p[3])*stride/scale;
    if(!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h)) continue;
    float x0=std::clamp(cx-w/2,0.f,float(original.width)),y0=std::clamp(cy-h/2,0.f,float(original.height));
    float x1=std::clamp(cx+w/2,0.f,float(original.width)),y1=std::clamp(cy+h/2,0.f,float(original.height));
    cv::Rect box(int(x0),int(y0),int(x1-x0),int(y1-y0));
    if(box.width<2 || box.height<2) continue;
    boxes.push_back(box);scores.push_back(score);classes.push_back(cls);
  }
  std::vector<int> keep;
  // Suppression is class-aware; overlapping people and vehicles are distinct.
  cv::dnn::NMSBoxesBatched(boxes,scores,classes,confidence_,nms_,keep,1.f,max_results_);
  std::vector<Detection> result;
  for(int index:keep) {
    if(int(result.size())>=max_results_) break;
    result.push_back({0,classes[index],scores[index],cv::Rect2f(boxes[index])});
  }
  return result;
}
void FastTracker::init(const cv::Mat& image,const cv::Rect2f& box) {
  box_=cv::Rect(box)&cv::Rect(0,0,image.cols,image.rows);
  if(box_.width<4 || box_.height<4) throw std::runtime_error("target box too small for KCF");
  tracker_=cv::TrackerKCF::create();size_=image.size();tracker_->init(image,box_);
  // KCF learns its initial template on the first update. Train on the seed,
  // rather than on a later frame that may already have moved.
  if(!tracker_->update(image,box_)) throw std::runtime_error("KCF initialization failed");
}
bool FastTracker::update(const cv::Mat& image,cv::Rect2f& box) {
  if(!tracker_ || image.size()!=size_ || !tracker_->update(image,box_)) return false;
  if((box_&cv::Rect(0,0,image.cols,image.rows)).area()!=box_.area()) return false;
  box=cv::Rect2f(box_);return true;
}
}
