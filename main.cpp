#include <iostream>
#include   <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include <opencv2/imgproc/imgproc.hpp> // add in version 0.3

#include "betaslam/config.h"
#include "betaslam/vo.h"
#include "betaslam/frame.h"
#include <boost/timer.hpp>


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
	//cout << i++ << rgb_time <<  " " << depth_time << " " << atof ( rgb_time.c_str())<< endl;
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
    betaslam::Camera::Ptr camera(new betaslam::Camera); //or write func createcamera in class
    betaslam::VO::Ptr vo(new betaslam::VO);
    
     // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.3);
    cv::Point3d cam_pos( 0, -1.0, -0.5 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );
    
    for(int i=0; i<rgb_files.size(); ++i) {
     
	Mat color = cv::imread(rgb_files[i]);
	Mat depth = cv::imread(depth_files[i], -1);
	
	betaslam::Frame::Ptr pFrame = betaslam::Frame::createFrame();
	pFrame->camera_ = camera; pFrame->color_ = color;  pFrame->depth_ = depth; pFrame->time_stamp_ = rgb_times[i];
	
	boost::timer timer;
	vo->addFrame(pFrame);
	
	cout << "VO time of one frame" << timer.elapsed() << endl;
	
	if ( vo->state_ == betaslam::VO::LOST )
            break;
	
	
	
        SE3 Twc = pFrame->Tcw_.inverse();
        
        // show the map and the camera pose 
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Twc.rotation_matrix()(0,0), Twc.rotation_matrix()(0,1), Twc.rotation_matrix()(0,2),
                Twc.rotation_matrix()(1,0), Twc.rotation_matrix()(1,1), Twc.rotation_matrix()(1,2),
                Twc.rotation_matrix()(2,0), Twc.rotation_matrix()(2,1), Twc.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Twc.translation()(0,0), Twc.translation()(1,0), Twc.translation()(2,0)
            )
        );
	
	Mat img_show = color.clone();
	for(auto &pt: vo->map_->map_points_) {
	    Vector2d pixel = pFrame->camera_->w2p(pt.second->pos_, pFrame->Tcw_);
	    cv::circle(img_show, cv::Point2f(pixel(0,0), pixel(1,0)), 3, cv::Scalar(0,0,255), 2);
	}
	
        cv::imshow("image", img_show);//color );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
	
	
	//cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
	//cv::imshow("Display Image", color);
	//cv::waitKey(0);
	//if (i == 2) break;
    }
    
    
    
    
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
