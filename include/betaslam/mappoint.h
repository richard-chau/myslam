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
  int grayscale_;
  
  static long factory_id_;
  list<Frame*> observed_frames_;
  
  MapPoint() :id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0),
			  matched_times_(0) {}

  MapPoint(long int id, const Vector3d& pos, const Vector3d& norm, Frame *frame=nullptr, const Mat& desc=Mat())
    :id_(id), pos_(pos), norm_(norm), good_(true), visible_times_(1), matched_times_(1), desc_(desc), grayscale_(-1){
      observed_frames_.push_back(frame);
    }

  MapPoint(long int id, const Vector3d& pos, const Vector3d& norm, Frame *frame=nullptr, float grayscale=0)
    :id_(id), pos_(pos), norm_(norm), good_(true), visible_times_(1), matched_times_(1), grayscale_(grayscale) {
      desc_ = Mat();
      observed_frames_.push_back(frame);
    }
    
  inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
  
  static MapPoint::Ptr createMapPoint() {
    return Ptr(new MapPoint(factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0), nullptr, Mat() ));
  }
  
  static MapPoint::Ptr createMapPoint(const Vector3d& pos_world, const Vector3d& norm, 
    const Mat& descriptor, Frame* frame
  ) {
    return Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
  }
  
  static MapPoint::Ptr createMapPointWithGrayScale(const Vector3d& pos_world, const Vector3d& norm, 
    float grayscale, Frame* frame
  ) {
    return Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, grayscale));
  }
};
  
}

#endif // MAPPOINT_H