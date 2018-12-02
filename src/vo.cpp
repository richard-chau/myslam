#include "betaslam/vo.h"
#include "betaslam/config.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace betaslam {
  


 
void VO::extractKAD()
{
  //std::vector<cv::KeyPoint> keypoints_curr;
  //Mat desc_curr, desc_ref;
  //std::vector<DMatch> matches;
  //cout << "param" << Config::get_param("number_of_features") << Config::get_param("scale_factor") << Config::get_param("level_pyramid" ) << endl;
  cv::Ptr<cv::ORB> orb_ = cv::ORB::create ( Config::get_param("number_of_features"), 
					    Config::get_param("scale_factor"), 
					    Config::get_param("level_pyramid" ));
  //cout << curr_->color_.rows << curr_ ->color_.row(0) << endl;
  orb_->detect ( curr_->color_, keypoints_curr_);
  orb_->compute ( curr_->color_, keypoints_curr_, desc_curr_ );
}

void VO::featureMatching()
{
  
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(desc_curr_, desc_ref_, matches);
  cout << Config::get_param("match_ratio")<<" " <<matches.size() <<endl;
  float min_dis = std::min_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2){
    return m1.distance < m2.distance;
  })->distance;
  //int min_dis = 99999;
  //for(int i=0; i<matches.size(); ++i) {
  //  if (matches[i].distance < min_dis) 
  //    min_dis = matches[i].distance;
  //}
  
  cout << Config::get_param("match_ratio")<<endl;
  // Sort matches by score
  //std::sort(matches.begin(), matches.end()); 
  // Remove not so good matches
  //const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  //matches.erase(matches.begin()+numGoodMatches, matches.end());
  
  goodmatches_.clear();
  
  for(auto &m: matches) {
      if (m.distance < max(min_dis * Config::get_param("match_ratio"), float(30.0))) {
	  goodmatches_.push_back(m);
      }
  }
  cout << min_dis << "good matches: " << goodmatches_.size() << endl;
}

void VO::PosePnP()
{
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;
  for(auto m: goodmatches_) {
      pts3d.push_back(pts_3d_ref_[m.queryIdx]);
      pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
  }
  
  //Mat mat0 = (Mat_(3, 3) << 10, 9, 8, 7, 6, 5, 4, 3, 2)
  //Mat mat1(3,3,CV_8UC3,Scalar(100,150,255));
  //Mat mat2 = Mat::eye(3,3,CV_8UC3);//cout << "mat2=" << endl << mat2 << endl << endl;
  //Mat mat3 = Mat::ones(3,3,CV_8UC1);//cout << "mat3=" << endl << mat3 << endl << endl;
  //Mat mat4 = Mat::zeros(3,3,CV_32F);//cout << "mat4=" << endl << mat4 << endl << endl;

  Mat K = ( cv::Mat_<double>(3,3) << ref_->camera_->fx_, 0, ref_->camera_->cx_, 
	    0, ref_->camera_->fy_,  ref_->camera_->cy_, 0, 0, 1 );
  Mat rvec, tvec, inliers; //rotation vector 
  cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
  num_inliers_ = inliers.rows;
  cout << "pnp inliers" << num_inliers_ << endl;
  Tcr_ = SE3(SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
	     Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
	 );
  
}

void VO::updateRef()
{
  //pts_3d_ref_
  //desc_ref_
  pts_3d_ref_.clear(); //vector
  desc_ref_ = Mat(); //Mat()
  //cout << keypoints_curr_.size() << "fsad " << endl;
  for(int i=0; i<keypoints_curr_.size(); ++i) {
      double d = ref_->findDepth(keypoints_curr_[i]);
      if (d > 0) {
	Vector3d p_cam = ref_->camera_->p2c(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
	pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
	desc_ref_.push_back(desc_curr_.row(i));
	//cout << desc_curr_.row(i) << endl;;
	
      }
  }
}


bool VO::checkgoodPose()
{
// check if the estimated pose is good
    if ( num_inliers_ < Config::get_param("min_inliers") )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = Tcr_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}


bool VO::addFrame(Frame::Ptr frame)
{
  
  
  switch(state_) {
    case INITIALIZING: {
     state_ = OK;
     curr_ = ref_ = frame;
     extractKAD();
     updateRef();
     break;
    }
    case OK: {
      
     curr_ = frame;
     extractKAD();
    
     featureMatching();
     
     PosePnP();
     
     if (checkgoodPose()) {
       curr_->Tcw_ = Tcr_ * ref_->Tcw_;
       ref_ = curr_;
       updateRef();
       num_lost_ = 0;
       
     } else {
	++num_lost_;
	if (num_lost_ > Config::get_param("max_num_lost")) {
	    state_ = LOST;
	}
	return false;
     }
     
      break;
    }
    case LOST:
    {
     cout << "VO has lost!" << endl;
      break;
    }
    
    return true;
  }
}

  
}