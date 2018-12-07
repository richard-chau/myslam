#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

#include <opencv2/core/core.hpp>
using cv::Mat;

#include <vector>
#include <list>
#include <memory>
#include <iostream>
#include <string>
#include <set>
#include <unordered_map>
#include <map>

#include <random>
#include <algorithm>
using namespace std;
#endif