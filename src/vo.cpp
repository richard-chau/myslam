#include "betaslam/vo.h"
#include "betaslam/config.h"
#include "betaslam/g2o_types.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/sparse_optimizer.h>

#include <boost/timer.hpp>


namespace betaslam {
  


 
void VO::extractKAD()
{
  //std::vector<cv::KeyPoint> keypoints_curr;
  //Mat desc_curr, desc_ref;
  //std::vector<DMatch> matches;
  //cout << "param" << Config::get_param("number_of_features") << Config::get_param("scale_factor") << Config::get_param("level_pyramid" ) << endl;
  
  //cout << curr_->color_.rows << curr_ ->color_.row(0) << endl;
  boost::timer timer;
  orb_->detect ( curr_->color_, keypoints_curr_);
  orb_->compute ( curr_->color_, keypoints_curr_, desc_curr_ );
  cout<<"extract keypoints and compute descriptors: "<<timer.elapsed() <<endl;
  
}

void VO::featureMatching()
{
  
  vector<cv::DMatch> matches;
  //cv::BFMatcher matcher(cv::NORM_HAMMING);
  //cv::FlannBasedMatcher matcher( new cv::flann::LshIndexParams(5,10,2));
  //matcher.match(desc_curr_, desc_ref_, matches); //wrong!!
  //matcher.match(desc_ref_, desc_curr_, matches);
  
  Mat desc_map;
  vector<MapPoint::Ptr> candidate;
  for(auto &item : map_->map_points_) {
      MapPoint::Ptr p = item.second;
      if (curr_->isInFrame(p->pos_)){
	  p->visible_times_ += 1;
	  desc_map.push_back(p->desc_);
	  candidate.push_back(p);
      } //else ?? TODO
  }
  matcher.match(desc_map, desc_curr_, matches);
  
  //cout << Config::get_param("match_ratio")<<" " <<matches.size() <<endl;
  float min_dis = std::min_element(matches.begin(), matches.end(),
				   [](const cv::DMatch& m1, const cv::DMatch& m2){
    return m1.distance < m2.distance;
  })->distance;
  //int min_dis = 99999;
  //for(int i=0; i<matches.size(); ++i) {
  //  if (matches[i].distance < min_dis) 
  //    min_dis = matches[i].distance;
  //}
  
  
  // Sort matches by score
  //std::sort(matches.begin(), matches.end()); 
  // Remove not so good matches
  //const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  //matches.erase(matches.begin()+numGoodMatches, matches.end());
  
  //goodmatches_.clear();
  matched_2d.clear();
  matched_inmap.clear();
  for(auto &m: matches) {
      if (m.distance < max(min_dis * Config::get_param("match_ratio"), float(30.0))) {
	  //goodmatches_.push_back(m);
	  matched_inmap.push_back(candidate[m.queryIdx]);
	  matched_2d.push_back(m.trainIdx);
      }
  }
  //cout << min_dis << " " << Config::get_param("match_ratio") << "good matches: " << goodmatches_.size() << endl;
}

void VO::PosePnP()
{
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;
  //cout << goodmatches_.size() << "pnp " << endl;
  //for(cv::DMatch m: goodmatches_) {
      //pts3d.push_back(pts_3d_ref_[m.queryIdx]);
      //pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
      //cout << keypoints_curr_[m.trainIdx].pt<<endl;
  //}
  for(auto i: matched_2d) {
      pts2d.push_back(keypoints_curr_[i].pt);
  }
  for(MapPoint::Ptr &p: matched_inmap) {
    Vector3d pos_tmp = p->pos_;
    pts3d.push_back(cv::Point3f( pos_tmp(0,0), pos_tmp(1,0), pos_tmp(2,0) ) ); // TODO
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
  Tcw_ = SE3(SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
	     Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
	 );
  
  
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2> > Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block *solver_ptr = new Block(linearSolver);
  
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(
    Tcw_.rotation_matrix(), Tcw_.translation())
  );
  optimizer.addVertex(pose);
  
  for(int i=0; i<inliers.rows; ++i) {
      int index = inliers.at<int>(i,0);
      EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
      edge->setId(i); edge->setVertex(0, pose);//pose again
      edge->camera_ = curr_->camera_.get();
      
      edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
      edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
      edge->setInformation(Eigen::Matrix2d::Identity());
      optimizer.addEdge(edge);
      
      matched_inmap[index]->matched_times_++;/////!!!!!!!!!!!!!!
  }
  
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  
  Tcw_ = SE3 (
    pose->estimate().rotation(),
    pose->estimate().translation()
  );
}

// void VO::updateRef()
// {
//   //pts_3d_ref_
//   //desc_ref_
//   pts_3d_ref_.clear(); //vector
//   desc_ref_ = Mat(); //Mat()
//   //cout << keypoints_curr_.size() << "fsad " << endl;
//   for(int i=0; i<keypoints_curr_.size(); ++i) {
//       double d = ref_->findDepth(keypoints_curr_[i]);
//       if (d > 0) {
// 	Vector3d p_cam = ref_->camera_->p2c(
// 	  Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
// 	pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
// 	desc_ref_.push_back(desc_curr_.row(i));
// 	//cout << desc_curr_.row(i) << endl;;
// 	//cout << p_cam << endl;
// 	
//       }
//   }
// }


bool VO::checkgoodPose()
{
// check if the estimated pose is good
    if (VO::methods==0 && num_inliers_ < Config::get_param("min_inliers") )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = (ref_->Tcw_ * Tcw_.inverse()).log();// Trc //version 0.2: Tcr_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}



bool VO::addFrame(Frame::Ptr frame)
{
  if(VO::methods == 1) 
    return addFrame_ds(frame);
  
  switch(state_) {
    case INITIALIZING: {
     state_ = OK;
     curr_ = ref_ = frame;
     extractKAD();
     //updateRef();
     //addMapPoints(1);
     addKeyFrame();

     break;
    }
    case OK: {
      
     curr_ = frame;
     curr_->Tcw_ = ref_->Tcw_;// different from 0.2!!
     extractKAD();
    
     featureMatching();
     
     PosePnP();
     
     if (checkgoodPose()) { //If Tcw_ is good, then assign it to curr_
       //curr_->Tcw_ = Tcr_ * ref_->Tcw_;
       curr_->Tcw_ = Tcw_;
       updateMap(); // addpoints() in this function
       //ref_ = curr_; // only occurs in addKeyFrame() and INITIALIZING
       //updateRef();
       num_lost_ = 0;
       
       if (checkKeyFrame())  addKeyFrame(); //cache the features & descripters in this frame
       
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



void VO::addKeyFrame() {
  
  
  if (map_->keyframes_.empty()) {
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
      //cout << i << "debug" << endl;
	double d = ref_->findDepth(keypoints_curr_[i]); //ref_ not curr_->?????
	if (d < 0)
	  continue;
	Vector3d pw = ref_->camera_->p2w(
	    Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
	    curr_->Tcw_, d);
	Vector3d n = pw - ref_->getCamCenter();
	n.normalize();
	MapPoint::Ptr map_pt;
	
	 map_pt = MapPoint::createMapPoint(
	    pw, n, desc_curr_.row(i).clone(), curr_.get()//raw pointer to frame!!!!
	  );
	
	map_->insertMapPoint(map_pt);
    }
  }
  
  map_->insertKeyFrame(curr_);
  cout << "add one frame" << endl;
  ref_ = curr_;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   Correct?
}

bool VO::checkKeyFrame()
{
  //SE3 Trc = ref_->Tcw_.inverse(); //different from 0.2
  SE3 Trc = ref_->Tcw_ * Tcw_.inverse();
  Sophus::Vector6d d = Trc.log();
  Vector3d trans = d.head<3>();
  Vector3d rot = d.tail<3>();
  //cout <<"check keyframe: norm of t and r"<< trans.norm() << " " << rot.norm() << endl;
  //return true;
  
  if (rot.norm() > Config::get_param("keyframe_rotation") || trans.norm() > Config::get_param("keyframe_translation"))
    return true;
  
  return false;
}


void VO::addMapPoints() //init_=0 in declaration
{
  //matched_2d, keypoints_curr_
  //filter those matched, add unmatched
  
  vector<bool> matched(keypoints_curr_.size(), false);
  
   for ( int index: matched_2d )
	  matched[index] = true;
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i])   
            continue;
	double d = ref_->findDepth(keypoints_curr_[i]); //ref_ not curr_->?????
	//here keypoints are pixel indices produced by orb_. 
	//d may be refined by triangulation.
	if (d < 0)
	  continue;
	Vector3d pw = ref_->camera_->p2w(
	    Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
	    curr_->Tcw_, d);
	Vector3d n = pw - ref_->getCamCenter();
	n.normalize();
	
	MapPoint::Ptr map_pt;
	if (VO::methods == 0) {
	  map_pt = MapPoint::createMapPoint(
	    pw, n, desc_curr_.row(i).clone(), curr_.get()//raw pointer to frame!!!!
	  );
	}/* else {
	  int y = keypoints_curr_[i].pt.y;
	  int x = keypoints_curr_[i].pt.x;
	  float grayscale = curr_->gray_.ptr<ushort>(y)[x];
	  map_pt = MapPoint::createMapPointWithGrayScale(
	    pw, n, grayscale, curr_.get()//raw pointer to frame!!!!
	  );
	}*/
	
	map_->insertMapPoint(map_pt);
    }
}


void VO::updateMap()
{
  //addMapPoints();
  //cout<<"map points:start "<<map_->map_points_.size()<<endl;
  // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
	//cout<<"map points: llll"<<map_->map_points_.size()<<endl;
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //cout<<"map points: "<<map_->map_points_.size()<<endl;
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //cout<<"map points: "<<map_->map_points_.size()<<endl;
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        
        //cout<<"map points: "<<map_->map_points_.size()<<endl;
        //if ( iter->second->good_ == false )
        //{
            // TODO try triangulate this map point 
        //}
        iter++;
    }
    
    if ( matched_2d.size()<100 )
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VO::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}



void VO::addKeyFrame_ds() {
  //TODO:
  
  static int cnt = 0;
  
  
  if (map_->map_points_.empty() == false) //must do this!!
    map_->map_points_.clear();
  //map_.reset();
  
  extractInitPt();
  
  //cout <<  float(curr_->gray_.ptr<uchar> (36) [35]) << "sfa df" << endl;

  for ( int i=0; i<keypoints_curr_.size(); i++ )
  {
      double d = ref_->findDepth(keypoints_curr_[i]); //ref_ not curr_->?????
      //cout << i << " " << d << " debug" << ref_->depth_.rows <<" "<<ref_->depth_.cols<< endl;  
      //cout << keypoints_curr_[i].pt.x << keypoints_curr_[i].pt.y  <<double(curr_->gray_.ptr<uchar>(36)[35])<< endl;
      if (d < 0)
	continue;
      Vector3d pw = ref_->camera_->p2w(
	  Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
	  curr_->Tcw_, d);
      Vector3d n = pw - ref_->getCamCenter();
      n.normalize();
      
      
      //cout << i << "debug" << double(curr_->gray_.ptr<uchar>(35)[36]) << endl;
      int y = keypoints_curr_[i].pt.y;
      int x = keypoints_curr_[i].pt.x;
      
      float grayscale = double(curr_->gray_.ptr<uchar>(y)[x]);
      //cout << keypoints_curr_[i].pt.x << keypoints_curr_[i].pt.y  <<double(curr_->gray_.ptr<uchar>(y)[x]) << endl;
      //cout << "grayscale" << grayscale << endl;
      MapPoint::Ptr map_pt = MapPoint::createMapPointWithGrayScale(
	pw, n, grayscale, curr_.get()//raw pointer to frame!!!!
      );
      //cout << i << "debug" << y <<" " << x << " " << n << " " << grayscale << endl;
      
      map_->insertMapPoint(map_pt);
      //cout << i << "debug" << endl;
  }
  cout << "new map:" << ++cnt << " " <<  map_->map_points_.size() << endl;
 
  
  map_->insertKeyFrame(curr_);
  cout << "add one frame" << endl;
  ref_ = curr_;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   Correct?
}

void VO::extractInitPt()
{
  cv::Mat gray = curr_->gray_;
  //cv::cvtColor(curr_->color_, gray, cv::COLOR_BGR2GRAY);
  //curr_->gray_ = gray;
  cout << "init map" << endl;
  keypoints_curr_.clear();
  //measurements.clear();
  for(int y=10; y<gray.rows-10; ++y){
    for(int x=10; x<gray.cols - 10; ++x) 
       {
	Vector2d delta(gray.ptr<uchar>(y)[x+1] - gray.ptr<uchar>(y)[x-1],
		       gray.ptr<uchar>(y+1)[x] - gray.ptr<uchar>(y-1)[x]
		 );
	if (delta.norm() < 50) 
	  continue;
	
	//keypoints_curr_.push_back(cv::KeyPoint(cv::Point2f(x, y), 0)); //size
	
 	double d = double(curr_->depth_.ptr<ushort>(y)[x]) / curr_->camera_->depth_scale_;
	//just like Frame::findDepth(cv::KeyPoint) but depth must exist here
	//cout << d << endl;
	if (d == 0) 
	  continue;
	
	keypoints_curr_.push_back(cv::KeyPoint(cv::Point2f(x, y), 0)); //size
	
	Vector3d p3d = curr_->camera_->p2c(Vector2d(x, y), d);
	float grayscale = float ( gray.ptr<uchar> (y) [x] );
	//measurements.push_back(Measurement(p3d, grayscale));
	
      }
      
  }
  //cout <<  float ( gray.ptr<uchar> (36) [35] ) << endl;
  //cout <<  float(gray.ptr<uchar> (36) [35]) << "sfa df" << endl;
  //   measurements.clear();
//   for(auto &item : map_->map_points_) {
//       MapPoint::Ptr p = item.second;
//       if (curr_->isInFrame(p->pos_)){
// 	  p->visible_times_ += 1;
// 	  Vector3d c_tmp = ref_->camera_->w2c(p->pos_, ref_->Tcw_);
// 	  measurements.push_back(Measurement(c_tmp, p->grayscale_));
//       } //else ?? TODO
//   }
  
  cout << "keypoint size:" <<  keypoints_curr_.size() << endl;
}


void VO::PoseDirect()
{
  
//   for (auto ptr=measurements.begin(); ptr!=measurements.end(); ++ptr) {
//   //for(auto &item : map_->map_points_) {
//     //MapPoint::Ptr p = item.second;
//       MapPoint::Ptr p = *ptr;
//       if (curr_->isInFrame(p->pos_)){
// 	  p->visible_times_ += 1;
//       } else {
// 	map_->map_points_.erase(p);
// 	measurements.erase(p);
//       }
//   }
  
  measurements.clear();
//   for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
//     {
// 	MapPoint::Ptr p = iter->second;
// 	if (curr_->isInFrame(p->pos_)){
//  	  p->visible_times_ += 1;
// 	  //Vector3d p3d = curr_->camera_->p2c(Vector2d(x, y), d);
// 	//float grayscale = float ( gray.ptr<uchar> (y) [x] );
// 	  measurements.emplace_back(ref_->camera_->w2c(p->pos_, ref_->Tcw_) ,p->grayscale_);
//        } else {
//             iter = map_->map_points_.erase(iter);
//             continue;
//         }
//     }
  
  
  std::vector<int> tobedeleted;
  for(auto &item : map_->map_points_) {
     MapPoint::Ptr p = item.second;
     if (curr_->isInFrame(p->pos_)){
 	  p->visible_times_ += 1;
	  //Vector3d p3d = curr_->camera_->p2c(Vector2d(x, y), d);
	//float grayscale = float ( gray.ptr<uchar> (y) [x] );
	  measurements.emplace_back(ref_->camera_->w2c(p->pos_, ref_->Tcw_) ,p->grayscale_);
       } else {
	 //cout << "3 4" <<endl;
	//map_->map_points_.erase(item.first);
	 tobedeleted.push_back(item.first);
     }
  }
  for(int i: tobedeleted) {
      map_->map_points_.erase(i);
  }
  
  //cout << measurements.size() << endl;
  if (measurements.size() < Config::get_param("min_ds_map_cnt")) {
      addKeyFrame_ds();
  }
  
  boost::timer timer;
     
     
    
    //Mat K = ( cv::Mat_<double>(3,3) << ref_->camera_->fx_, 0, ref_->camera_->cx_, 
	//    0, ref_->camera_->fy_,  ref_->camera_->cy_, 0, 0, 1 );
   
    Eigen::Matrix3f K;
    K << ref_->camera_->fx_, 0, ref_->camera_->cx_, 
	   0, ref_->camera_->fy_,  ref_->camera_->cy_, 0, 0, 1;
    //<<fx,0.f,cx,0.f,fy,cy,0.f,0.f,1.0f;
    //cout <<"mm size:" <<  measurements.size() << K << endl;
     
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock; 
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    //optimizer.setVerbose( true );

    
    
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    
    pose->setEstimate ( g2o::SE3Quat ( Tcw_.rotation_matrix(), Tcw_.translation() ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
            m.pw_,
            K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), &(curr_->gray_) //pointer!
        );
        edge->setVertex ( 0, pose );
        edge->setMeasurement ( m.grayscale );
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        edge->setId ( id++ );
        optimizer.addEdge ( edge );
    }
    //cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    //Tcw_ = pose->estimate();
    //cout << "pose es:" << pose->estimate().matrix() << endl;
    Tcw_ = SE3 (
    pose->estimate().rotation(),
    pose->estimate().translation()
  );
    //cout << "pose es:" << Tcw_ << " " << pose->estimate().rotation().toRotationMatrix() << endl;
    //Tcw_ = Tcw_ * ref_->Tcw_;

  
     cout<<"PoseDirectinner: "<<timer.elapsed() << endl; 
}


void VO::updateMap_ds()
{
  //TODO:
  //add new points into the map. How to differentiate these points, and maintain the size
  cv::Mat gray = curr_->gray_;
  
  for(int y=10; y<gray.rows-10; ++y){
    for(int x=10; x<gray.cols - 10; ++x) 
       {
	Vector2d delta(gray.ptr<uchar>(y)[x+1] - gray.ptr<uchar>(y)[x-1],
		       gray.ptr<uchar>(y+1)[x] - gray.ptr<uchar>(y-1)[x]
		 );
	if (delta.norm() < 50) 
	  continue;
	
	//keypoints_curr_.push_back(cv::KeyPoint(cv::Point2f(x, y), 0)); //size
	
 	double d = double(curr_->depth_.ptr<ushort>(y)[x]) / curr_->camera_->depth_scale_;
	//just like Frame::findDepth(cv::KeyPoint) but depth must exist here
	if (d == 0) 
	  continue;
	
	keypoints_curr_.push_back(cv::KeyPoint(cv::Point2f(x, y), 0)); //size
	
	Vector3d p3d = curr_->camera_->p2c(Vector2d(x, y), d);
	float grayscale = float ( gray.ptr<uchar> (y) [x] );
	//measurements.push_back(Measurement(p3d, grayscale));
	//use map
      }
  }
  
  cout<<"map points:"<<map_->map_points_.size()<<endl;

}  


bool VO::checkKeyFrame_ds()
{
  if (map_->map_points_.size() < Config::get_param("min_ds_map_cnt")) {
      return true;
  } 
  
  SE3 Trc = ref_->Tcw_ * Tcw_.inverse();
  Sophus::Vector6d d = Trc.log();
  Vector3d trans = d.head<3>();
  Vector3d rot = d.tail<3>();
  cout <<"check keyframe: norm of t and r"<< trans.norm() << " " << rot.norm() << endl;
  //return true;
  
  if (rot.norm() > Config::get_param("keyframe_rotation") || trans.norm() > Config::get_param("keyframe_translation")) {
    cout << "Warning: Direct semi-dense has abnormal rotation/translation magnitude. Rebuild the map" << endl;
    return true;
  }  
  return false;
}

  
bool VO::addFrame_ds(Frame::Ptr frame)
{
  static int cnt = 0;
  
  switch(state_) {
    case INITIALIZING: {
     state_ = OK;
     curr_ = ref_ = frame;
     curr_->Tcw_ = Tcw_;
     ref_->Tcw_ = Tcw_;
     addKeyFrame_ds();
     break;
    }
    case OK: {
     ++cnt; 
      
     curr_ = frame;
     curr_->Tcw_ = ref_->Tcw_;
     cout<<"map points:"<<map_->map_points_.size()<<endl;
      boost::timer timer;
      PoseDirect();
      cout<<"PoseDirect: "<<timer.elapsed() << endl; 
     
     //PosePnP();
     //curr_->Tcw_ = Tcw_;
     
     
     if (cnt < 2 || checkgoodPose()) { //If Tcw_ is good, then assign it to curr_
       //cout << "get" << endl;
       //curr_->Tcw_ = Tcr_ * ref_->Tcw_;
       curr_->Tcw_ = Tcw_;
       //updateMap_ds(); // addpoints() in this function
       //ref_ = curr_; // wrong!!
       //updateRef();
       //num_lost_ = 0;
       
       //if (checkKeyFrame_ds())  {
	// addKeyFrame_ds(); //cache the features & descripters in this frame
	 //..//ref_ = curr_;
       //}
       
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




int betaslam::VO::methods = 0;
}