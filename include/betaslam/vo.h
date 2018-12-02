#ifndef VO_H
#define VO_H

#include "betaslam/common_include.h"
#include "betaslam/frame.h"

namespace betaslam {

class VO{
public:
  typedef shared_ptr<VO> Ptr;

  
  Frame::Ptr curr_, ref_;
  
  std::vector<cv::KeyPoint> keypoints_curr_;
  Mat desc_curr_, desc_ref_;
  std::vector<cv::DMatch> goodmatches_;
  
  vector<cv::Point3f> pts_3d_ref_;
  int num_inliers_;
  int num_lost_; 
  SE3 Tcr_;
  
  enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
  VOState state_;
  
  VO(): state_(INITIALIZING), ref_(nullptr), curr_(nullptr), num_inliers_(0), num_lost_(0){}
  ~VO(){}
  bool addFrame(Frame::Ptr frame);
  
protected:
  void extractKAD(); //keypoint and descriptors
  void featureMatching();
  void PosePnP();
  void updateRef();
  bool checkgoodPose();
};
}

#endif