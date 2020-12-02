//
// Created by tom on 16/11/2020.
//

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H


#include "myslam/commoninclude.h"
#include "myslam/Map.h"
#include <chrono>
#include "myslam/dataset.h"
#include "myslam/config.h"
#include "myslam/backend.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam
{
    class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        /// constructor with config file
        VisualOdometry(std::string &config_path);

        /**
         * do initialization things before run
         * @return true if success
         */
        bool Init();

        /**
         * start vo in the dataset
         */
        void Run();

        /**
         * Make a step forward in dataset
         */
        bool Step();

        /// 获取前端状态
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        bool inited_ = false;
        std::string config_file_path_;
        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

         //dataset
        Dataset::Ptr dataset_ = nullptr;
    };
/*    class VisualOdometry
{
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING = -1,
            OK = 0,
            LOST
        };
        VOState state_;//current VO status
        Map::Ptr map_;//map with all frams and map points
        Frame::Ptr ref_;//reference frame
        Frame::Ptr curr_; //current frame

        cv::Ptr<cv::ORB> orb_;  //orb detector and computer
        vector<cv::Point3f> pts_3d_ref_; //keypoints in reference frame
        vector<cv::KeyPoint> Keypoints_curr_; //keypoints in curent frame
        Mat descriptors_curr_; //descriptor in current frame
        Mat descriptors_ref_; //decriptor in reference frame
        vector<cv::DMatch> feature_matches_;

        SE3 T_c_r_estimated_; // the estimated pose of current frame
        int num_inliers_; //number of inlier features in icp
        int num_lost_; // number of lost times

        //parameters
        int num_of_features_; //number of features
        double scale_factor_; //scale in image pyrramid
        int level_pyramid_; //number of pyramid levels
        float match_ratio_; //ratio for selecting good matches
        int max_num_lost_; //max number of continuous lost times
        int min_inliers_; //minimum inliers

        double key_frame_min_rot;//minimal rotation of two_keyframes
        double key_frame_min_trans; //minimal translation of two key-frames
        double  map_point_erase_ratio_; // remove map point ratio,新增

    public: //functions
        VisualOdometry();

        ~VisualOdometry();


        bool addFrame(Frame::Ptr frame);     //add a new frame

    protected:
        //inner operation
        void extractKeyPoint();

        void computeDescriptors();

        void featureMatching();

        void poseEstimationPnP();

        void optimizeMap(); //新增,对地图进行优化，包括删除不在视野内的点，在匹配数量减少时添加新点

        void setRef3DPoints();

        void addKeyFrame();

        bool checkEstimatedPose();

        bool checkKeyFrame();
};*/
}
#endif //VISUAL_ODOMETRY_H

