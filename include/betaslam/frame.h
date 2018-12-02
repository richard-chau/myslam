#ifndef FRAME_H
#define FRAME_H

#include "betaslam/common_include.h"
#include "betaslam/camera.h"

namespace betaslam{
class Frame {
public:
  typedef std::shared_ptr<Frame> Ptr;
  int id_;
  double time_stamp_;
  SE3 Tcw_;
  Mat color_, depth_;
  bool is_key_frame;
  
  Camera::Ptr camera_;
};
}

#endif