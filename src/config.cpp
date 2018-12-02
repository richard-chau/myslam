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
    }
  }
}