//
// Created by tom on 26/11/2020.
//


#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/Map.h"
#include "myslam/viewer.h"

namespace myslam {

    Frontend::Frontend() {
        gftt_ =
                cv::GFTTDetector::create(Config::get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::get<int>("num_features_init");
        num_features_ = Config::get<int>("num_features");
    }

    bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
        current_frame_ = frame;

        switch (status_) {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:

            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Track() {

        if (last_frame_) {
            // (velocity is no change) relative_motion change of (n-2)(n-1) , so current frame= v* last frame
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());//Tcl * Tlw = Tcw

        }

        int num_track_last = TrackLastFrame();//use LK flow tracking feacture from last and current using left frame
        tracking_inliers_ = EstimateCurrentPose();//vertex  camera pose , edge  feature in camera axies

        if (tracking_inliers_ > num_features_tracking_)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else
            if (tracking_inliers_ > num_features_tracking_bad_)
            {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
            }
            else
            {
            // lost
            status_ = FrontendStatus::LOST;
            }
            //如果跟踪点数目太少, 需要关键帧，根据左右匹配点并三角化为地图点

        InsertKeyframe();
        //relative_motion = Tcw*Tlw^-1 =Tcw*Twl =Tcl last frame to current frame
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
        //display current frame
        if (viewer_) viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::InsertKeyframe() {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
            // still have enough features, don't insert keyframe
            return false;
        }
        // current frame is a new keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_); // control number of key frame

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();
        DetectFeatures();  // detect new features based on having features

        // track in right image
        FindFeaturesInRight();
        // triangulate map points
        TriangulateNewPoints();
        // update backend because we have a new keyframe
        backend_->UpdateMap();

        if (viewer_) viewer_->UpdateMap();

        return true;
    }

    void Frontend::SetObservationsForKeyFrame() {
        for (auto &feat : current_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);//add stability of feature
        }
    }

    int Frontend::TriangulateNewPoints() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr) {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                        camera_left_->pixel2camera(
                                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                     current_frame_->features_left_[i]->position_.pt.y)),
                        camera_right_->pixel2camera(
                                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                     current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                            current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                            current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose() {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex let pose of current frame add to top point vectex
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Mat33 K = camera_left_->K();

        // edges is data with the error is mp , vertex is feature in camera
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;  //only optimize pose no mappoint
        std::vector<Feature::Ptr> features;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp) {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                        new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                        toVec2(current_frame_->features_left_[i]->position_.pt));//set measurement point(feature)
                //信息矩阵，也是协方差矩阵用来约束各个维度的可信度
                //可信度与金字塔的层级有关
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);//核函数  防止异常值  
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;//自由度为2 的情况下 卡方分布95%以上可信度的阈值
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();//measurement - estimate
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);//set current frame is useless
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);//the edge is useful.
                };

                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;
        // Set pose and outlier
        current_frame_->SetPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

        for (auto &feat : features) {
            if (feat->is_outlier_) {
                feat->map_point_.reset();
                feat->is_outlier_ = false;  // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame() {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_) {
            if (kp->map_point_.lock()) {
                // use project point
                auto mp = kp->map_point_.lock();//get matched map points of left side feature
                auto px =
                        camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                       //let map point project on current plane by curent frame pose
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            } else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }
        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
                last_frame_->left_img_, current_frame_->left_img_, kps_last,
                kps_current, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);//keypoint diameter
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }
  //1.detect features from left image(add new feature),
  // matching from the right(optional flow).
  // if feature good(triangulation) tracking it
    bool Frontend::StereoInit() {
        int num_features_left = DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_) {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success) {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_) {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures() {
        //the white mask for searching feature 255 is white
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        //make a 10x10 rectangle for every landmasks
        for (auto &feat : current_frame_->features_left_) {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints) {
            current_frame_->features_left_.push_back(
                    Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight() {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        // kp represent the first ptr of features of left
        for (auto &kp : current_frame_->features_left_) {
            kps_left.push_back(kp->position_.pt);//let every features push into kps_left
            auto mp = kp->map_point_.lock();//3D(map) point matched feature
            if (mp) {
                // use projected points as initial guess ,means mp piont can be triangulation
                auto px =
                        camera_right_->world2pixel(mp->pos_, current_frame_->Pose());//3d point project into right frame
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            } else {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }

        }

        std::vector<uchar> status;
        Mat error;

        //形参 (前一幅图像,后一幅图像,前一幅图像中想要跟踪的点集,后一幅图像中计算得到的对应点集,status 1 is sucsuss,记录每个特征点的误差, 金字塔的层数,
        // 迭代停止条件，默认设置为30次迭代或者阈值0.01,默认值为0，OPTFLOW_USE_INITIAL_FLOW)
        cv::calcOpticalFlowPyrLK(
                current_frame_->left_img_, current_frame_->right_img_, kps_left,
                kps_right, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW);
        int num_good_pts = 0;
        //record correct matching point from LK method
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                //(pt： 特征点的坐标,size2：特征点的大小,angle：特征点的角度,response:特征点的响应强度
                // octave：特征点所在的金字塔的哪一组class_id：特征点的分类)
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;//a flag of keypoint
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            } else {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " <<num_good_pts << " current_frame";


        return num_good_pts;
    }

    bool Frontend::BuildInitMap() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        //scan left image features
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i] == nullptr) continue;//no exit feature in right imagwe will skip
            // create map point from triangulation
            std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                            Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                 current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                            Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                 current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();
             //pworld[2]>0 means having the depth value.
            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
        }
        current_frame_->SetKeyFrame();//keyframe id ++
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap();
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks<< " map points";
        return true;
    }
    bool Frontend::Reset() {
        LOG(INFO) << "Reset is not implemented. ";
        return true;
    }
}