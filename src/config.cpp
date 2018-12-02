#include "betaslam/config.h"

namespace betaslam {
  shared_ptr<Config> Config::config_ = nullptr;
  
  void Config::create() {
    cout << "created" << endl;
    if ( config_ == nullptr ) {
      config_ = shared_ptr<Config>(new Config);
      config_->dataset_dir  = "/home/winter/Desktop/slam/slambook-master/rgbd_dataset_freiburg1_xyz";
      config_->kmap["camera_fx"] = 517.3;
      config_->kmap["camera_fy"] = 516.5;
      config_->kmap["camera_cx"] = 325.1;
      config_->kmap["camera_cy"] = 249.7;
      config_->kmap["camera_depth_scale"] = 5000.0;
      
      config_->kmap["number_of_features"] = 500;
      config_->kmap["scale_factor"] = 1.2;
      config_->kmap["level_pyramid"] = 8;
      config_->kmap["match_ratio"] = 2;
      config_->kmap["max_num_lost"] = 10;
      config_->kmap["keyframe_rotation"] = 0.1;
      config_->kmap["keyframe_translation"] = 0.1;
      config_->kmap["min_inliers"] = 10;
    }
  }
}