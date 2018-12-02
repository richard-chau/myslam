#ifndef CONFIG_H
#define CONFIG_H

#include "betaslam/common_include.h"

namespace betaslam {
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
  
  
  static string get_dataset(const std::string &key) {
      return Config::config_->dataset_dir;
  }
  
  static float get_param(const std::string &key) {
    return Config::config_->kmap[key];
  }
 };  
}


#endif