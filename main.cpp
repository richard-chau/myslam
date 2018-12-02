#include <iostream>
#include <config.h>



int main(int argc, char **argv) {
    myslam::Config::create();
    string dataset_dir = myslam::Config::get<string> ("dataset_dir");
    cout << dataset_dir << endl;
  
  
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
