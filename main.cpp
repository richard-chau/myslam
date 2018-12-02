#include <iostream>
#include   <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "config.h"


int main(int argc, char **argv) {
    betaslam::Config::create();
    string dataset_dir = betaslam::Config::get_dataset ("dataset_dir");
    float camera_fx = betaslam::Config::get_param("camera_fx");
    cout << dataset_dir << "sfdads " << camera_fx << endl;
    ifstream fin( dataset_dir + "/associate.txt");
    cout << dataset_dir + "/associate.txt" << endl;
    if (!fin) {
      throw std::invalid_argument("no associate file input");
    }
    
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    int i=0;
    while( !fin.eof() ) {
	string rgb_time, rgb_file, depth_time, depth_file;
	fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
	cout << i++ << rgb_time <<  " " << depth_time << " " << atof ( rgb_time.c_str())<< endl;
	//rgb_times.push_back ( atof ( rgb_time.c_str() ) ); //if we use this, the last one is 0
        //depth_times.push_back ( atof ( depth_time.c_str() ) );
	
	if (fin.good() == false) // line 792: xx, xx; but line 793: empty
	  break;
	
	rgb_times.push_back(stod(rgb_time));//atof(rgb.to_cstr())); //return double
	depth_times.push_back(stod(depth_time));
	rgb_files.push_back(dataset_dir + "/" + rgb_file);
	depth_files.push_back(dataset_dir + "/" + depth_file);
    }
    
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    
    for(int i=0; i<rgb_files.size(); ++i) {
	Mat color = cv::imread(rgb_files[i]);
	Mat depth = cv::imread(depth_files[i], -1);
	
	
	
	//cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
	//cv::imshow("Display Image", color);
	//cv::waitKey(0);
	break;
    }
    
    
    
    
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
