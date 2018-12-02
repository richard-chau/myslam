cmake_minimum_required(VERSION 2.8)
project(betaslam)

set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

set( Sophus_INCLUDE_DIRS  "/home/winter/Desktop/slam/slambook-master/3rdparty/Sophus;/usr/local/include/eigen3" )
set( Sophus_LIBRARIES    "/home/winter/Desktop/slam/slambook-master/3rdparty/Sophus/build/libSophus.so" )

include_directories( "/usr/include/eigen3" )
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
)


include_directories( ${PROJECT_SOURCE_DIR}/include/ )
add_subdirectory(src)

add_executable( run_vo main.cpp )
target_link_libraries( run_vo betaslam )

