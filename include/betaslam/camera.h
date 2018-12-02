#ifndef CAMERA_H
#define CAMERA_H

#include "betaslam/common_include.h"

namespace betaslam {
 class Camera {
 public:
   typedef std::shared_ptr<Camera> Ptr;
   float fx_, fy_, cx_, cy_, depth_scale_;
   
   Camera();
   Camera(float fx, float fy, float cx, float cy, float depth_scale=0) : fx_(fx), fy_(fy),
	cx_(cx), cy_(cy), depth_scale_(depth_scale) {}
   ~Camera(){}
   
   Vector3d w2c (const Vector3d& p_w, const SE3& Tcw);
   Vector3d c2w (const Vector3d& p_c, const SE3& Tcw);
   Vector2d c2p (const Vector3d& p_c);
   Vector3d p2c (const Vector2d& p_p, double depth=1);
   Vector3d p2w (const Vector2d& p_p, const SE3& Tcw, double depth=1);
   Vector2d w2p (const Vector3d& p_w, const SE3& Tcw);
   
 };
  
}

#endif