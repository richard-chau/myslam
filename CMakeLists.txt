cmake_minimum_required(VERSION 2.8)
project(betaslam)

set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_STANDARD 14)
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

set( Sophus_INCLUDE_DIRS  "/home/winter/Desktop/slam/slambook-master/3rdparty/Sophus;/usr/local/include/eigen3" )
set( Sophus_LIBRARIES    "/home/winter/Desktop/slam/slambook-master/3rdparty/Sophus/build/libSophus.so" )
set( G2O_INCLUDE_DIRS "/home/winter/Desktop/slam/slambook-master/3rdparty/g2o" )


include_directories( "/usr/include/eigen3" )
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# 增加PCL库的依赖
#find_package(PCL 1.2 REQUIRED)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04
# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
# 添加头文件和库文件
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${PCL_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)


include_directories( ${PROJECT_SOURCE_DIR}/include/ )
add_subdirectory(src)

add_executable( run_vo main.cpp )
target_link_libraries( run_vo betaslam )

