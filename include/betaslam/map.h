#ifndef MAP_H
#define MAP_H

#include "betaslam/common_include.h"
#include "betaslam/frame.h"
#include "betaslam/mappoint.h"

namespace betaslam {
class Map {
public:
  typedef shared_ptr<Map> Ptr;
  Map(){};
  ~Map(){};
  
  unordered_map<long, MapPoint::Ptr> map_points_;
  unordered_map<long, Frame::Ptr> keyframes_;
  
  void insertKeyFrame(Frame::Ptr frame) {
      cout<<"Key frame size = "<<keyframes_.size()<<endl;
      if ( keyframes_.find(frame->id_) == keyframes_.end() )
	  keyframes_.insert( make_pair(frame->id_, frame) );
      else
	  keyframes_[ frame->id_ ] = frame;
    
  }
  
  
  void insertMapPoint(MapPoint::Ptr map_point){
      if (map_points_.find(map_point->id_) == map_points_.end())
	map_points_.insert({map_point->id_, map_point});
      else 
	map_points_[map_point->id_] = map_point;
  }
};
  
}

#endif