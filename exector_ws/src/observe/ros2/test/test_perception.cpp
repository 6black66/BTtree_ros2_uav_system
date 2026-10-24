#include <observe/perception.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#define REQUIRE(condition) do { if(!(condition)) throw std::runtime_error(#condition); } while(false)
int main(int argc,char** argv) {
  try {
    REQUIRE(argc>=2);
    cv::setNumThreads(2);
    const std::string dir=argv[1];
    observe::Detector detector(dir+"/object_detection_yolox_2022nov.onnx",dir+"/coco.names",0.45,0.45,100);
    int shape[]={1,8400,85};cv::Mat output(3,shape,CV_32F,cv::Scalar(0));
    // Two identical same-class boxes and a different class at the same location.
    auto put=[&](int row,int cls) {
      float* p=output.ptr<float>()+85*row;
      p[0]=10-row;p[1]=10;p[2]=std::log(4.f);p[3]=std::log(4.f);p[4]=0.9;p[5+cls]=0.9;
    };
    put(0,0);put(1,0);put(2,2);
    auto decoded=detector.decode(output,{640,640});REQUIRE(decoded.size()==2);
    REQUIRE(decoded[0].box.width==32);REQUIRE(decoded[0].box.x==64);
    REQUIRE(std::abs(decoded[0].confidence-0.81)<1e-5);
    REQUIRE(detector.label(0)=="person");REQUIRE(detector.label(2)=="car");
    bool rejected=false;
    try {detector.decode(cv::Mat(10,85,CV_32F),{640,640});} catch(const std::exception&) {rejected=true;}
    REQUIRE(rejected);
    observe::IdAssigner ids;
    std::vector<observe::Detection> a={{0,0,0.9,{20,20,40,40}},{0,2,0.8,{100,20,50,50}}};
    ids.assign(a,1,0.3,2);const auto person=a[0].id,car=a[1].id;
    std::vector<observe::Detection> b={{0,2,0.9,{102,20,50,50}},{0,0,0.8,{22,20,40,40}}};
    ids.assign(b,2,0.3,2);REQUIRE(b[0].id==car);REQUIRE(b[1].id==person);
    std::vector<observe::Detection> c={{0,0,0.9,{22,20,40,40}}};ids.assign(c,5,0.3,2);REQUIRE(c[0].id!=person);
    auto old=c[0].id;ids.reset();c[0].id=0;ids.assign(c,6,0.3,2);REQUIRE(c[0].id!=old);
    cv::Mat seed(240,320,CV_8UC3,cv::Scalar(40,40,40));
    cv::Mat patch(60,60,CV_8UC3);cv::RNG rng(2026);rng.fill(patch,cv::RNG::UNIFORM,0,255);
    patch.copyTo(seed(cv::Rect(80,80,60,60)));
    observe::FastTracker tracker;tracker.init(seed,{80,80,60,60});
    cv::Rect2f box;
    for(int i=1;i<=6;++i) {
      cv::Mat frame(seed.size(),seed.type(),cv::Scalar(40,40,40));patch.copyTo(frame(cv::Rect(80+i*2,80,60,60)));
      REQUIRE(tracker.update(frame,box));
    }
    REQUIRE(std::abs(box.x-92)<8);REQUIRE(!tracker.update(cv::Mat(120,160,CV_8UC3),box));
    REQUIRE(!tracker.update(cv::Mat(seed.size(),seed.type(),cv::Scalar(40,40,40)),box));
    for(int arg=2;arg<argc;++arg) {
      auto image=cv::imread(argv[arg]);REQUIRE(!image.empty());auto detections=detector.detect(image);REQUIRE(!detections.empty());
      std::cout<<argv[arg]<<":";
      for(const auto& d:detections) std::cout<<" "<<detector.label(d.class_id)<<"="<<d.confidence;
      std::cout<<"\n";
      if(std::string(argv[arg]).find("basketball")!=std::string::npos) {
        bool person_found=false;for(const auto& d:detections) if(d.class_id==0) person_found=true;REQUIRE(person_found);
      }
      if(std::string(argv[arg]).find("dog416")!=std::string::npos) {
        bool car_found=false;for(const auto& d:detections) if(d.class_id==2 || d.class_id==7) car_found=true;REQUIRE(car_found);
      }
    }
    std::cout<<"observe perception tests passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
}
