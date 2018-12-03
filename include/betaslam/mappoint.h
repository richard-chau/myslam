#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "betaslam/common_include.h"

namespace betaslam
{
class Frame;
class MapPoint {
public:
  typedef shared_ptr<MapPoint> Ptr;
  
  long int id_;
  bool good_;
  Vector3d pos_, norm_;
  Mat desc_;
  int matched_times_;
  int visible_times_;
  
  static long factory_id_;
  list<Frame*> observed_frames_;
  
  MapPoint() :id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0),
			  matched_times_(0) {}

  MapPoint(long int id, const Vector3d& pos, const Vector3d& norm, Frame *frame=nullptr, const Mat& desc=Mat())
    :id_(id), pos_(pos), norm_(norm), good_(true), visible_times_(1), matched_times_(1), desc_(desc){
      observed_frames_.push_back(frame);
    }
    
  inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
  
  static MapPoint::Ptr createMapPoint() {
    return Ptr(new MapPoint(factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0)) );
  }
  
  static MapPoint::Ptr createMapPoint(const Vector3d& pos_world, const Vector3d& norm, 
    const Mat& descriptor, Frame* frame
  ) {
    return Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
  }
};
  
}

#endif // MAPPOINT_H