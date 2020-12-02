#cmake_minimum_required(VERSION 3.0)
#project(myslam)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-O3")

# Eigen
#FIND_PACKAGE(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
# OpenCV
#FIND_PACKAGE(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus
#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})
#Pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})
# 添加Eigen头文件
#include_directories("/usr/include/eigen3")
#add_executable(camera camera.cpp)
#add_executable(config config.cpp)
#add_executable(frame frame.cpp)
#add_executable(visual_odometry visual_odometry.cpp)

#target_link_libraries( camera ${OpenCV_LIBS} ${Sophus_LIBRARIES})
#target_link_libraries( config ${OpenCV_LIBS} )
#target_link_libraries(camera config frame visual_odometry)

add_library( 0_1 SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        visual_odometry.cpp
        frontend.cpp
        dataset.cpp
        backend.cpp
        viewer.cpp
        )

target_link_libraries( 0_1
        ${THIRD_PARTY_LIBS}
        )