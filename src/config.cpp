#include "betaslam/config.h"

namespace betaslam {
  shared_ptr<Config> Config::config_ = nullptr;
  
  void Config::create() {
    cout << "created" << endl;
    if ( config_ == nullptr ) {
      config_ = shared_ptr<Config>(new Config);
      //config_->dataset_dir  = "/home/winter/Desktop/slam/slambook-master/ch8/data/data/";
      //config_->kmap["camera_fx"] = 518.0;//517.3; //for data 8
      //config_->kmap["camera_fy"] = 519.0;//516.5;
      //config_->kmap["camera_cx"] = 325.5;//325.1;
      //config_->kmap["camera_cy"] = 253.5;//249.7;
      //config_->kmap["camera_depth_scale"] = 1000;//5000.0;
      //config_->kmap["number_of_features"] = 500;
      
      config_->dataset_dir  ="/home/winter/Desktop/slam/slambook-master/rgbd_dataset_freiburg1_xyz";
      config_->kmap["camera_fx"] = 517.3;
      config_->kmap["camera_fy"] = 516.5;
      config_->kmap["camera_cx"] = 325.1;
      config_->kmap["camera_cy"] = 249.7;
      config_->kmap["camera_depth_scale"] = 5000.0;
      
      config_->kmap["number_of_features"] = 700;//800;//500;
      config_->kmap["scale_factor"] = 1.2;
      config_->kmap["level_pyramid"] = 4;//8;
      config_->kmap["match_ratio"] = 2;
      config_->kmap["max_num_lost"] = 10;
      config_->kmap["keyframe_rotation"] = 0.05; //min_rot
      config_->kmap["keyframe_translation"] = 0.1; //min_trans
      config_->kmap["min_inliers"] = 30;//10;
      config_->kmap["map_point_erase_ratio"]= 0.1;////0.5;
      
      config_->kmap["min_ds_map_cnt"] = 10000; //direct-semidense
    }
  }
}