#include "config.h"

namespace betaslam {
  shared_ptr<Config> Config::config_ = nullptr;
  
  void Config::create() {
    cout << "created" << endl;
    if ( config_ == nullptr )
      config_ = shared_ptr<Config>(new Config);
      config_->dataset_dir  = "/home/winter/Desktop/slam/slambook-master/rgbd_dataset_freiburg1_xyz";
      config_->kmap["camera_fx"] = 517.3;
  }
}