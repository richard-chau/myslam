#ifndef VO_H
#define VO_H

#include "betaslam/common_include.h"
#include "betaslam/frame.h"
#include "betaslam/map.h"
#include "betaslam/config.h"
#include <opencv2/features2d/features2d.hpp>

namespace betaslam {

class VO{
public:
  typedef shared_ptr<VO> Ptr;

  Map::Ptr map_;
  Frame::Ptr curr_, ref_;
  
  std::vector<cv::KeyPoint> keypoints_curr_;
  Mat desc_curr_;
  Mat desc_ref_;
  //std::vector<cv::DMatch> goodmatches_;
  std::vector<MapPoint::Ptr> matched_inmap;
  std::vector<int> matched_2d;
  
  vector<cv::Point3f> pts_3d_ref_;
  int num_inliers_;
  int num_lost_; 
  SE3 Tcw_;
  
  double map_point_erase_ratio_;
  
  enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
  VOState state_;
  
  cv::Ptr<cv::ORB> orb_; 
  cv::FlannBasedMatcher matcher;
  VO(): state_(INITIALIZING), ref_(nullptr), curr_(nullptr), num_inliers_(0), num_lost_(0),
	orb_(cv::ORB::create ( Config::get_param("number_of_features"), 
					    Config::get_param("scale_factor"), 
					    Config::get_param("level_pyramid" ))),
	matcher(new cv::flann::LshIndexParams(5,10,2)), map_(new Map),
	map_point_erase_ratio_(Config::get_param("map_point_erase_ratio")){}
  ~VO(){}
  bool addFrame(Frame::Ptr frame);
  
protected:
  void extractKAD(); //keypoint and descriptors
  void featureMatching();
  void PosePnP();
//void updateRef();
  bool checkgoodPose();
  
  void addKeyFrame();
  bool checkKeyFrame();
  void updateMap();
  void addMapPoints();
  double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
  
};
}

#endif