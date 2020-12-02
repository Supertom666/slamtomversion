//
// Created by tom on 16/11/2020.
//

#include "myslam/visualodometry.h"
#include <chrono>
#include "myslam/config.h"
#include "myslam/dataset.h"
namespace myslam
{
    VisualOdometry::VisualOdometry(std::string &config_path)
            : config_file_path_(config_path) {}

    bool VisualOdometry::Init() {
        // read from config file
        if (!Config::setParameterFile(config_file_path_)) {
            return false;
        }

        dataset_ =Dataset::Ptr(new Dataset(Config::get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->Init(), true);

        // create components and links
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdometry::Run() {
        while (1) {
            LOG(INFO) << "VO is running";
            if (!Step()) {
                //std::cout<<"1"<<std::endl ;
                break;
            }

        }

        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit";
    }

    bool VisualOdometry::Step() {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr) return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";

        return success;
    }
 /*   myslam::VisualOdometry::VisualOdometry():
    state_ ( INITIALIZING ), ref_ ( nullptr ) ,curr_ ( nullptr ), map_ ( new Map), num_lost_ ( 0 ), num_inliers_ ( 0 )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}
    VisualOdometry::~VisualOdometry()
    {

    }
    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch(state_)
        {
            case INITIALIZING:
            {
                state_ = OK ;
                curr_ = ref_ = frame;
                map_->insertKeyFrame(frame);
                //extract feature from first frame
                extractKeyPoint();
                computeDescriptors();
                //computer the 3d position of feature in ref frame
                setRef3DPoints();
                break;
            }
            case OK:
            {
                curr_ = frame;
                extractKeyPoint();
                computeDescriptors();
                featureMatching();
                poseEstimationPnP();
                if(checkEstimatedPose()) //a good estimation
                {
                    curr_-> T_c_w_ = T_c_r_estimated_ * ref_ -> T_c_w_;
                    ref_ = curr_ ;
                    setRef3DPoints();
                    num_lost_ = 0 ;
                    if (checkKeyFrame()) // is a jey_frame
                    {
                        addKeyFrame();

                    }
                }
                else //bad estimation due to various reasons
                {
                    num_lost_++;
                    if ( num_lost_ > max_num_lost_ )
                    {
                        state_ = LOST;

                    }
                    return false;

                };
                break;
            }
            case LOST:
            {
                cout<<"vo has lost." << endl;
                break;
            }
        }
        return true;
    }

    void VisualOdometry::extractKeyPoint()
    {

        orb_->detect (curr_->color_, Keypoints_curr_ );
    }

    void VisualOdometry::computeDescriptors()
    {
        orb_->compute ( curr_->color_, Keypoints_curr_, descriptors_curr_ );
    }

    void VisualOdometry::featureMatching()
    {
        // match desp_ref and desp_curr, use OpenCV's brute force match
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher ( cv::NORM_HAMMING );
        matcher.match ( descriptors_ref_, descriptors_curr_, matches );
        // select the best matches
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        feature_matches_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }
        cout<<"good matches: "<<feature_matches_.size()<<endl;
    }
    void VisualOdometry::setRef3DPoints()
    {
        // select the features with depth measurements
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
        for ( size_t i=0; i<Keypoints_curr_.size(); i++ )
        {
            double d = ref_->findDepth(Keypoints_curr_[i]);
            if ( d > 0)
            {
                Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(Keypoints_curr_[i].pt.x, Keypoints_curr_[i].pt.y), d );
                pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }
    void VisualOdometry::poseEstimationPnP()
    {
        // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for ( cv::DMatch m:feature_matches_ )
        {
            pts3d.push_back( pts_3d_ref_[m.queryIdx] );
            pts2d.push_back( Keypoints_curr_[m.trainIdx].pt );
        }

        Mat K = ( cv::Mat_<double>(3,3)<<
                ref_->camera_->fx_, 0, ref_->camera_->cx_,
                0, ref_->camera_->fy_, ref_->camera_->cy_,
                0,0,1
        );
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        num_inliers_ = inliers.rows;
        cout<<"pnp inliers: "<<num_inliers_<<endl;
        T_c_r_estimated_ = SE3(
                Sophus::SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
        );
    }
    bool VisualOdometry::checkEstimatedPose()
    {
        // check if the estimated pose is good
        if ( num_inliers_ < min_inliers_ )
        {
            cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }
    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = T_c_r_estimated_.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
            return true;
        return false;
    }

    void VisualOdometry::addKeyFrame()
    {
        cout<<"adding a key-frame"<<endl;
        map_->insertKeyFrame ( curr_ );
    }*/
}
