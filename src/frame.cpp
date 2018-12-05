#include "betaslam/frame.h"

namespace betaslam {
Frame::Frame(long int id, double time_stamp, SE3 Tcw, Camera::Ptr camera, Mat color, Mat depth, Mat gray):
  id_(id), time_stamp_(time_stamp), Tcw_(Tcw), camera_(camera), color_(color), depth_(depth), 
  gray_(gray){}
  
Frame::Ptr Frame::createFrame()
{
  static long factory_id = 0;
  //return new shared_ptr<Frame>();
  return Frame::Ptr(new Frame(factory_id++));
}

double Frame::findDepth(const cv::KeyPoint& kp)
{
  //return double(depth_.ptr<ushort>(y)[x]/camera_->depth_scale_;
  //some corner cases:
  int x = cvRound(kp.pt.x);
  int y = cvRound(kp.pt.y);
  ushort d = depth_.ptr<ushort>(y)[x];
  if (d != 0) {
      return double(d)/camera_->depth_scale_;
  }
  else {
      d = depth_.ptr<ushort>(y-1)[x];
      if (d != 0 ) return double(d)/camera_->depth_scale_;
      d = depth_.ptr<ushort>(y+1)[x];
      if (d != 0 ) return double(d)/camera_->depth_scale_;
      d = depth_.ptr<ushort>(y)[x-1];
      if (d != 0 ) return double(d)/camera_->depth_scale_;
      d = depth_.ptr<ushort>(y)[x+1];
      if (d != 0 ) return double(d)/camera_->depth_scale_;
  }
  return -1.0;
}
  
bool Frame::isInFrame(const Vector3d& p_w)
{
    Vector3d p_cam = camera_->w2c( p_w, Tcw_ );
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->w2p( p_w, Tcw_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 && pixel(0,0)<color_.cols && pixel(1,0)<color_.rows;

}


  
}