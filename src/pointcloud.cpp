#include "betaslam/pointcloud.h"


namespace betaslam {

PointCloud::Ptr image2PointCloud(Mat& rgb, Mat& depth, Camera::Ptr camera_)
{
  PointCloud::Ptr cloud (new  PointCloud);
  
  for (int y=0; y<depth.rows; y+=2) {
      for (int x=0; x<depth.cols; x+=2) {
	  ushort d = depth.ptr<ushort>(y)[x]; //char??
	  if (d == 0)
	    continue;
	  PointT p;
	  double depth = double(d) / camera_->depth_scale_;
	  p.x = ( x - camera_->cx_ ) * depth / camera_->fx_;
	  p.y = ( y - camera_->cy_ ) * depth / camera_->fy_;
	  p.z = depth; 
	  
	  p.b = rgb.ptr<uchar>(y)[x*3];
	  p.g = rgb.ptr<uchar>(y)[x*3+1];
	  p.r = rgb.ptr<uchar>(y)[x*3+2];
	  
	  cloud->points.push_back(p);
      }
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  
  return cloud;
}


Eigen::Isometry3d SE3_to_Eigen(SE3 &T_SE3)
{
  Eigen::Matrix3d r = T_SE3.rotation_matrix();
  Vector3d t = T_SE3.translation();
  
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd angle(r);
  T = angle;
  T(0,3) = t(0,0); 
  T(1,3) = t(1,0); 
  T(2,3) = t(2,0);
  return T;
}


PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, Frame::Ptr& curr_, SE3 T_SE3)
{
  Eigen::Isometry3d T = SE3_to_Eigen(T_SE3);
  
  PointCloud::Ptr newcloud = image2PointCloud( curr_->color_,curr_->depth_, curr_->camera_ );
  PointCloud::Ptr output(new PointCloud());
  pcl::transformPointCloud(*original, *output, T.matrix());
  *newcloud += *output;
  
  static pcl::VoxelGrid<PointT> voxel;
  double gridsize = Config::get_param("voxel_grid");
  voxel.setLeafSize(gridsize, gridsize, gridsize);
  voxel.setInputCloud(newcloud);
  PointCloud::Ptr tmp(new PointCloud());
  voxel.filter(*tmp);
  return tmp;
}

}
