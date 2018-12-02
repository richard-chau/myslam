#include "betaslam/camera.h"
#include "betaslam/config.h"

namespace betaslam {
  Camera::Camera() {
	fx_ = Config::get_param("camera_fx");
	fy_ = Config::get_param("camera_fy");
	cx_ = Config::get_param("camera_cx");
	cy_ = Config::get_param("camera_cy");
	depth_scale_ = Config::get_param("camera_depth_scale");
  }
  
Vector3d Camera::w2c(const Vector3d& p_w, const SE3& Tcw)
{
  return Tcw * p_w; 
}

Vector3d Camera::c2w(const Vector3d& p_c, const SE3& Tcw)
{
  return Tcw.inverse() * p_c;
}

Vector2d Camera::c2p(const Vector3d& p_c)
{
  return Vector2d (fx_ * p_c(0, 0) / p_c(2, 0) + cx_, fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vector3d Camera::p2c(const Vector3d& p_p, double depth)
{
  return Vector3d (
	  ( p_p(0,0) - cx_ ) * depth / fx_,
	  ( p_p(1,0) - cy_ ) * depth / fy_,
	  depth
	 );
}

Vector3d Camera::p2w(const Vector3d& p_p, const SE3& Tcw, double depth)
{
  return c2w(p2c(p_p, depth), Tcw);
}

Vector2d Camera::w2p(const Vector3d& p_w, const SE3& Tcw)
{
  return c2p(w2c(p_w, Tcw));
}
  
  
}