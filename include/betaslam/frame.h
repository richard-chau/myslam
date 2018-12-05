#ifndef FRAME_H
#define FRAME_H

#include "betaslam/common_include.h"
#include "betaslam/camera.h"

namespace betaslam{
class Frame {
public:
  typedef std::shared_ptr<Frame> Ptr;
  long int id_;
  double time_stamp_;
  SE3 Tcw_;
  Mat color_, depth_;
  bool is_key_frame;
  
  Camera::Ptr camera_;
  
  Mat gray_;
  
public:
  Frame(): id_(-1), time_stamp_(-1), camera_(nullptr) {}
  Frame( long id, double time_stamp=0, SE3 Tcw=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat(), Mat gray_=Mat());
  ~Frame(){}; //because camera_ is shared_ptr
  
  static Frame::Ptr createFrame();
  
  double findDepth(const cv::KeyPoint& kp);
  Vector3d getCamCenter() const {
    return Tcw_.inverse().translation(); //consider Twc * [0, 0, 1]
  }
  void setPose(const SE3& Tcw) {Tcw_ = Tcw; }
  
  bool isInFrame(const Vector3d& p_w);
};
}

#endif