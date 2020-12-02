//
// Created by tom on 13/11/2020.
//
#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "myslam/commoninclude.h"

namespace myslam  //命名空间 incase same name function
{
//pinhole RGB-D camera model
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr ; //shared_ptr<T> p1; 空智能指针，指向类型为T的对象
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
            baseline_ = 0;  // Camera intrinsics
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics; //camera instrinsics

    Camera();
    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }
    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }
    //coordinate transforms: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth =1  );
    Vector3d pixel2world( const Vector2d& p_p, const SE3& T_c_w, double depth =1  );
    Vector2d world2pixel( const Vector3d& p_w, const SE3& T_c_w );
};
}
#endif // CAMERA_H


