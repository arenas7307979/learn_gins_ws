#pragma once

// #define DEBUG_BE
#include "thread"
#include <condition_variable>
#include <iostream>
// must to be include utility.h that with macros info.
#include "slam_vio/utility.h"
#include "start_slam_sys/slam_param_reader.h"
#include "slam_vio/opticalflow/corner_tracker.h"
#include "opencv_display_ui/draw_slam_result.h"

#ifdef USE_CERES_VIO
#include "slam_vio/rgbd_TBA.h"
#else
#include "slam_vio/vsolver_rgbd_encoder_TBA.h"
#endif

#if USE_LOCAL_FUSION
#include "../local_fusion/relocalization.h"
#endif


#if USE_PDR_CORE
#include "pdr_core/A_PDR.h"
#endif

class System {
public:
    System(const std::string& config_file);
    // System(const std::string& config_file);
    ~System();

    void ResetSeq();
    void PushImages(const cv::Mat &img_l, const cv::Mat &depth_image, double timestamp);
    void PushImuData(const Eigen::Vector3d &gyr, const Eigen::Vector3d &acc, double timestamp);
    void SetPoseSavingPath(const std::string save_path);
    //void PushWheelOdomData(const Eigen::Vector2d &wheel_vlvr, double timestamp);
    
#ifdef USE_DISPLAY
    //----- callback function
    //pose callback function for display
    inline void SubVIOTwc(std::function<void(double, const Sophus::SE3d &, const Eigen::Vector3d &)>callback)
    {
        pub_vio_Twc = (callback);
    }

    inline void SubCalibrResult(std::function<void(double, const Eigen::Matrix<double, 10, 1>&)>callback)
    {
        pub_vio_ts_fcxcy_Td_Tbc = (callback);
    }

    inline void SubPQVSeq(std::function<void(double, const Eigen::Matrix<double, 10, 1>&, uint8_t)>callback)
    {
        pub_vio_Twc_seq = (callback);
    }

    inline void SubTrackingImg(std::function<void(double, const cv::Mat &)> callback)
    {
        pub_tracking_img = (callback);
    }

    inline void SubMappoint(std::function<void(double, const std::vector<Eigen::Vector3d> &)> callback)
    {
        pub_vio_x3Dc = (callback);
    }

#if USE_PDR_CORE
    inline void SubPDR_Result(std::function<void(const Eigen::Matrix<double, 11, 1> &)> callback)
    {
        pub_pdr_path = callback;
    }
#endif

#if INIT_DEBUG_SAVE_POSE_STATE
    inline void SubSlidingWindowPose(std::function<void(double, const InitDebugPose &)> callback)
    {
        pub_vio_sliding_window_Twc = (callback);
    }
#endif

#endif

    //wheel instrinsic info, only for ROS.main converter
    double wheel_rL, wheel_rR, wheel_b;
    Sophus::SE3d Tbc;     //for predict KLT
    Sophus::SE3d Twc_vio; //Twc_cur of vio

#if USE_LOCAL_FUSION
    RelocalizationPtr local_reloc;
    void RelocProcess();
    std::thread reloc_thread;
    std::mutex mtx_reloc;
    std::condition_variable cv_reloc;
    std::deque<VsolverRGBDEnCoderTBA::KeyFrameConstPtr> reloc_buffer_keyframe;
    // std::deque<std::pair<double, Sophus::SE3d>> reloc_buffer_cur_frame;
#endif

//protected:

#ifdef USE_DISPLAY
    //----- callback function
    std::function<void(double, const std::vector<Eigen::Vector3d> &)> pub_vio_x3Dc;             //pub cur 3d points
    std::function<void(double, const Sophus::SE3d &, const Eigen::Vector3d &)> pub_vio_Twc;     //pose / velocity
    std::function<void(double, const Eigen::Matrix<double, 10, 1> &, uint8_t)> pub_vio_Twc_seq; //pose(6) / velocity(3) / seq(1)
    std::function<void(double, const Eigen::Matrix<double, 10, 1> &)> pub_vio_ts_fcxcy_Td_Tbc;  //time_stamp(1) / fcxcy(3) / Td(1) / extrinsic para(6)
    std::function<void(double, const cv::Mat &)> pub_tracking_img;

#if USE_PDR_CORE
    std::function<void(const Eigen::Matrix<double, 11, 1> &)> pub_pdr_path; //pub cur 3d points
#endif

#if INIT_DEBUG_SAVE_POSE_STATE
    std::function<void(double, const InitDebugPose &)> pub_vio_sliding_window_Twc;
#endif
#else
    DrawSLAMResultPtr draw_map_cv;
#endif

    using TrackingConstFramePtr = CornerTracker::FrameConstPtr;
    using TrackingFramePtr = CornerTracker::FramePtr;

    std::string tum_save_path;

    //Rotation prediction from raw gyro data.
    void Only_R_Predict(std::vector<Eigen::Vector3d>& gyro, std::vector<Eigen::Vector3d>& acc, Sophus::SE3d &Tcl, 
    double cam0_prev_time, double cam0_cur_time);

    //read sensor info.
    void InitCameraModelParameters();
    camodocal::CameraPtr cam_s; // camera [left cam]
    camodocal::CameraPtr cam_m; // camera [right cam]

    //slam_param_reader
    ParamReaderPtr parameter_reader;

    //sensor info buffer
    //ForFrontEndPredict u, v
    std::deque<Eigen::Vector3d> front_end_buffer_gyr;
    std::deque<Eigen::Vector3d> front_end_buffer_acc;
    std::deque<double> front_end_buffer_imu_t; //[sec+nse]sec
    
    //ForBackEndOptimize
    std::deque<Eigen::Vector3d> buffer_gyr;
    std::deque<Eigen::Vector3d> buffer_acc;
    std::deque<double> buffer_imu_t; //[sec+nse]sec
    std::deque<Eigen::Vector2d> buffer_wheelvlvr;
    std::deque<double> buffer_wheel_t;//[sec+nse]sec

    // FeatureTrackerPtr feature_tracker
    std::thread frontend_thread;
    std::deque<std::tuple<cv::Mat, cv::Mat, double>> frontend_buffer_img;

    //frontend mutex / Process
    std::mutex mtx_frontend;
    std::condition_variable cv_frontend;

    //FrontEndProcess
    void FrontEndProcess();
    bool front_end_first_frame = true; //b_first_frame
    CornerTrackerPtr cam_s_tracking;
    std::map<uint64_t, std::deque<cv::Point2f>> prev_optical_flow_history; //for show tracking results
    
    //FrontEndProcess freq control for pulish to backend
    bool PUB_THIS_FRAME = false;
    bool first_input_flag = true;
    double FREQ;
    double first_image_time;
    int pub_count = 1;

    //Backend mutex / Process
    bool InitState = true;
    std::atomic<bool> backend_busy;
    std::atomic<bool> backend_wake_up{false};
    std::atomic<bool> frontend_wake_up{false};
    std::thread backend_thread;
    std::mutex mtx_backend;
    std::condition_variable cv_backend;
    std::deque<std::pair<TrackingConstFramePtr, cv::Mat>> backend_buffer_img;
    virtual void BackEndProcess();
#ifdef USE_CERES_VIO
    RGBDTBAPtr backend;
#else
    VsolverRGBDEnCoderTBAPtr backend;
#endif

    //slam_BE_last_frame
    double last_frame_timestamp = -1.0;
    //slam_FE_last_frame
    double front_end_last_frame_time = -1.0;
    //slam pose_graph time interval, Avoid add too many frames in a short time (local fusion)
    double last_frame_fusion = -1.0;

    //for saving VIO info for logoutput.
    std::vector<std::pair<double, Sophus::SE3d>> Twc_vec_log;
    std::vector<std::pair<int, int>> depth_num_and_avg_count;
    std::vector<Eigen::Matrix<double, 9, 1>> vel_acc_gyro_bias; //size align with Twc_vec_log


#if USE_PDR_CORE
    Sophus::SE3d Tvio_pdr;
    PDR_RunPtr pdr_core_run;
    std::vector<std::pair<double, Sophus::SE3d>> PDR_Twc_vec;
    std::mutex mtx_pdr_info;
#endif
    //newest VIO Twb/Bias/Timestamp for PDR prediction
    //Eigen::Matrix<double, 19, 1> vio_info_to_pdr; //time, tx ty tz. qx qy qz qw, vx, vy, vz, bg_xyz, ba_xyz, avg_depth_feature(tracking_life), number_depth
     VsolverRGBDEnCoderTBA::Result last_vio_frame;
};
using SystemPtr = std::shared_ptr<System>;
using SystemConstPtr = std::shared_ptr<const System>;
