#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include "betaslam/common_include.h"
//#include "betaslam/camera.h"
#include "betaslam/config.h"
#include "betaslam/frame.h"

namespace betaslam{

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

Eigen::Isometry3d SE3_to_Eigen(SE3 &T_SE3);

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, Camera::Ptr camera_);

// joinPointCloud 
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, Frame::Ptr& curr_, SE3 T) ;
}

#endif