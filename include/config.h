#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

namespace myslam {
class Config {
private:
  static std::shared_ptr<Config> config_;
  Config() {} //singleton mode
  
  std::string dataset_dir;
  std::unordered_map<string, float> kmap;
  
public:
  ~Config(){}
  
  static void create();
  //static void create() {
  //  if ( config_ == nullptr )
  //    config_ = shared_ptr<Config>(new Config);
  //    config_->dataset_dir  = "/home/winter/Desktop/slam/slambook-master/rgbd_dataset_freiburg1_xyz";
  //    config_->kmap["camera_fx"] = 517.3;
  //}
  
  template<typename T> 
  static T get(const std::string &key) {
    if (key == "dataset_dir")
      return Config::config_->dataset_dir;
    //else 
      //return Config::config_->kmap(key);
  }
  
};  
}


#endif