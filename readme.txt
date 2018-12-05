Hello, this is my repo for learning slam

Branch 0.1: 
Basic Class for a visual odometry, including class Camera, class Frame, class VO. Eigen, Sophus, cv::Mat is used to express the tramsform of cooridinates. cv::orb, cv::BFM is used to do feature extraction and descriptor matching. 

Branch 0.2:
Add the function of bundle adjustment of PnP generated pose in G2O. Extend a class in G2O and construct the vertices and edges. Replace cv::BFM with cv:flann. 

Branch 0.3: 
Add local map for visual odometry. Avoid simple pair-wise matching, instead maintain a map (descriptors cache). Add new struct map & mappoint. 
Update map with deleting and adding points. Maintain a good map size so that achieve a good speed. 
Currently 0.01-0.03 s one frame.
 
