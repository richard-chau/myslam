Hello, this is my repo for learning slam

Branch 0.1: 
Basic Class for a visual odometry, including class Camera, class Frame, class VO. Eigen, Sophus, cv::Mat is used to express the tramsform of cooridinates. cv::orb, cv::BFM is used to do feature extraction and descriptor matching. 

Branch 0.2:
Feature based Method added.
Add the function of bundle adjustment of PnP generated pose in G2O. Extend a class in G2O and construct the vertices and edges. Replace cv::BFM with cv:flann. 

Branch 0.3: 
Add local map for visual odometry. Avoid simple pair-wise matching, instead maintain a map (descriptors cache). Add new struct map & mappoint. 
Update map with deleting and adding points. Maintain a good map size so that achieve a good speed. 
Currently 0.01-0.03 s one frame.

Branch 0.4:
Directed Method added. Direct Semi-dense Method with local map. Fix bugs and update maps from time to time. 
Currently 0.04-0.09 s one frame. Mainly because of too many points thus over 20,000 edges in G2O. Remain to be optimized.
