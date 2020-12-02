//
// Created by tom on 14/11/2020.
//

#ifndef FRAME_H
#define FRAME_H
#include "myslam/commoninclude.h"
#include "myslam/camera.h"
namespace myslam
{
    //forward declare
struct MapPoint;
struct Feature;

struct Frame{
public: //data members
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame

    std::mutex pose_mutex_;          // Pose数据锁
    double time_stamp_{};            //when it is recorded
    SE3 pose_;                       //transform from world to camera Tcw 形式Pose
    //Camera::Ptr camera_;             // pinehole RGB-D camera ,model
    //Mat color_, depth_;            //color and depth image
    cv::Mat left_img_, right_img_;   // stereo images
    bool is_keyframe_ = false;       // 是否为关键帧

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;
public: //data members

    Frame() {}


    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);


    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配并键帧id
    void SetKeyFrame();

    /// 工厂构建模式，分配id
    static std::shared_ptr<Frame> CreateFrame();
};

}
#endif //INC_0_1_FRAME_H
