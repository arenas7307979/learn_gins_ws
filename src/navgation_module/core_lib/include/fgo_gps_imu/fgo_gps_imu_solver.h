#pragma once

#include <chrono>
#include <thread>

#include <slam_vio/sophus_extra.h>

// slam parameter
// #include "slam_vio/integration_base.h"
#include "slam_vio/bull_integration_base.h"
#include "start_slam_sys/slam_param_reader.h"

// our V-solver
#include "slam_vio/Vsolver/problem.h"
#include "slam_vio/Vsolver/vertex_bias.h"
#include "slam_vio/Vsolver/vertex_pose.h"
#include "slam_vio/Vsolver/vertex_speed.h"
#include "slam_vio/Vsolver/vertex_speedbias.h"
#include "slam_vio/Vsolver/edge_bull_imu.h"
#include "slam_vio/Vsolver/edge_imu_last_bias.h"
#include "slam_vio/Vsolver/edge_gnss.h"

//for ceres solver
#include "slam_vio/ceres/local_parameterization_se3.h"
#include "slam_vio/ceres/ceres_bull_imu_factor.h"
#include "slam_vio/ceres/gnss_ceres_factor.h"
#include "ob_gins/factors/marginalization_se3_order_factor.h"
#include "ob_gins/factors/pose_parameterization.h"

// for ba_solver template version
#include <ba_solver/graph_optimizor/edge/edge_gnss.hpp>
#include <ba_solver/graph_optimizor/edge/edge_imu_preintegration.hpp>
#include <ba_solver/graph_optimizor/edge/edge_last_imu_bias.hpp>
#include <ba_solver/graph_optimizor/kernel_function.hpp>
#include <ba_solver/graph_optimizor/problem.hpp>
#include <ba_solver/graph_optimizor/vertex/ba_vertex_bias.hpp>
#include <ba_solver/graph_optimizor/vertex/ba_vertex_se3.hpp>
#include <ba_solver/graph_optimizor/vertex/ba_vertex_speed.hpp>

using Scalar = double; // 方便切换浮点数精度

const double FGO_WGS84_WIE = 7.2921151467E-5;      // 地球自转角速度
const double FGO_WGS84_F = 0.0033528106647474805;  // 扁率
const double FGO_WGS84_RA = 6378137.0000000000;    // 长半轴a
const double FGO_WGS84_RB = 6356752.3142451793;    // 短半轴b
const double FGO_WGS84_GM0 = 398600441800000.00;   // 地球引力常数
const double FGO_WGS84_E1 = 0.0066943799901413156; // 第一偏心率平方
const double FGO_WGS84_E2 = 0.0067394967422764341; // 第二偏心率平方

class FGO_GpsIMU_SOLVER
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //Tool GNSS
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
    const double STD_stable_Th[2] = {
        0.25, 1.0}; //判断准静态的条件,加表模值标准差，最大最小值
    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r =
            atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    static double gravity(const Eigen::Vector3d &blh) {

        double sin2 = sin(blh[0]);
        sin2 *= sin2;
        return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
               blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
    }

    static Eigen::Matrix3d cne(const Eigen::Vector3d &blh) {
        double coslon, sinlon, coslat, sinlat;

        sinlat = sin(blh[0]);
        sinlon = sin(blh[1]);
        coslat = cos(blh[0]);
        coslon = cos(blh[1]);

        Eigen::Matrix3d dcm;
        dcm(0, 0) = -sinlat * coslon;
        dcm(0, 1) = -sinlon;
        dcm(0, 2) = -coslat * coslon;

        dcm(1, 0) = -sinlat * sinlon;
        dcm(1, 1) = coslon;
        dcm(1, 2) = -coslat * sinlon;

        dcm(2, 0) = coslat;
        dcm(2, 1) = 0;
        dcm(2, 2) = -sinlat;

        return dcm;
    }

    static double RN(double lat) {
        double sinlat = sin(lat);
        // std::cout << "FGO_WGS84_E1="<< FGO_WGS84_RA<< std::endl;
        // std::cout << "FGO_WGS84_RA="<< FGO_WGS84_E1<< std::endl;
        return FGO_WGS84_RA / sqrt(1.0 - FGO_WGS84_E1 * sinlat * sinlat);
    }

    static Eigen::Vector3d blh2ecef(const Eigen::Vector3d &blh)
    {
        double coslat, sinlat, coslon, sinlon;
        double rnh, rn;

        coslat = cos(blh[0]);
        sinlat = sin(blh[0]);
        coslon = cos(blh[1]);
        sinlon = sin(blh[1]);

        // std::cout << "RN(blh[0])=" << RN(blh[0]) <<std::endl;
        // std::cout << "rn + blh[2]=" << rn + blh[2] <<std::endl;

        rn = RN(blh[0]);
        rnh = rn + blh[2];

        return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * FGO_WGS84_E1) * sinlat};
    }


    //Input  lat(radius), long(radius), height(meter)
    //[Output is ENU coordinate, ori version : cn0e.transpose() * (ecef1 - ecef0) is NED coordinate.
    static Eigen::Vector3d global2local(const Eigen::Vector3d &origin, const Eigen::Vector3d &global)
    {
        Eigen::Vector3d ecef0 = blh2ecef(origin);
        Eigen::Matrix3d cn0e = cne(origin);

        Eigen::Vector3d ecef1 = blh2ecef(global);
        Eigen::Vector3d tmp_ned, tmp_enu;

        // std::cout << "ecef1=" << ecef1 <<std::endl;
        // std::cout << "ecef0=" << ecef0 <<std::endl;
        // std::cout << "cn0e.transpose()=" << cn0e.transpose() <<std::endl;

        tmp_ned = cn0e.transpose() * (ecef1 - ecef0);
        tmp_enu.x() = tmp_ned.y();
        tmp_enu.y() = tmp_ned.x();
        tmp_enu.z() = -tmp_ned.z();
        return tmp_enu;
    }

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum State {
        Gw_NEED_INIT,
        Acc_Need_INIT_Rwb,
        WAIT_FILL_WINDOW,
        TIGHTLY_GINS
    };

    enum Measurement {
        IMU_ONLY,
        PDR_IMU,
        GNSS_IMU,
        PDR_GNSS_IMU,
    };

    enum MarginTypeGNSS {
        MARGIN_SECOND_NEW,
        MARGIN_OLD,
        MARGIN_INIT
    };

    struct GNSSFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GNSSFrame() {
            ori_blh.setZero();
            p_wb.setZero();
            v_wb.setZero();
            ba.setZero();
            bg.setZero();
            station2cur_blh_local_data.setZero();
            q_wb = Sophus::SO3d();
            gnss_llh_variance.x() = 1;
            gnss_llh_variance.y() = 1;
            gnss_llh_variance.z() = 2;
        }
        virtual ~GNSSFrame() {
        }
        double timestamp;
        bool is_zupt_frame = false;
        bool have_gnss_info = false;

        // Frame state------------------
        Sophus::SO3d q_wb;
        Eigen::Vector3d p_wb;
        Eigen::Vector3d v_wb;
        Eigen::Vector3d ba;
        Eigen::Vector3d bg;
        double gnss_imu_td = 0; // time delay

        // GNSS Measurement---
        Eigen::Vector3d ori_blh; // Lat, long, attitude
        Eigen::Vector3d station2cur_blh_local_data; // Pw_gnss(~imu for phone platforrm), ENU coordinate.
        Eigen::Vector3d gnss_llh_variance;
        // PDR Measurement---
        Sophus::SE3d pose_Twpdr_b; // init translation value (100000000, 100000000, 100000000)
        bool pdr_sucess = false;

        // IMU-----------
        std::vector<Eigen::Vector3d> v_gyr, v_acc, v_mag, v_baro; // maybe changed by ZUPT
        std::vector<double> v_imu_timestamp;
        BullIntegrationBasePtr imupreinte;
    };
    using GNSSFramePtr = std::shared_ptr<GNSSFrame>;
    using GNSSFrameConstPtr = std::shared_ptr<const GNSSFrame>;
    
    struct GNSSResult {
        State state;
        GNSSFramePtr cur_frame_result;
    };

    FGO_GpsIMU_SOLVER();
    FGO_GpsIMU_SOLVER(ParamReaderPtr& slam_parameter);
    FGO_GpsIMU_SOLVER(double gyr_arw, double acc_vrw, double gyr_bias_std, double acc_bias_std, int window_size_gnss);

    ~FGO_GpsIMU_SOLVER();

    // FGO-GNSS and IMU
    void Reset();
    double Get_time_offset() const;
    GNSSResult ProcessGNSSIMU(GNSSFramePtr &frame);
    void PredictLastFrameOnly(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    double gravity_magnitude;

    Eigen::Vector3d Gw; //導航座標系的重力 (only using between the imu_factor, The rotate
    // effect has been removed from the residual term?)
private:
    /**
   * @brief Static Average acc_value estimate init Rw'w, which aligned with the gravity direction
   * @param frame info
   * @return 0 : not enough number acc data, 1 : not stable average acc, 2 : sucees Init
   */
    int InitEstAccRwb(GNSSFramePtr &frame);

    /**
   * @brief imu intergration and state update
   * @param ref_frame last_frame
   */
    void PredictNextGNSSFrameImu(const GNSSFramePtr &ref_frame, GNSSFramePtr &cur_frame);

    /**
   * @brief Using zupt_acc_norm vector estimate std and average to check whether is static movement.
   * @return 0 : not static, 1 : static
   */
    bool IMURawCheckZUPT();

    /**
   * @brief copy data info to double pointer container only for save BA result after solved.
   */
    void data2double();

    /**
   * @brief SolveFGO
   */
    void SolveFGO();
    void SlidingWindowSecondNew();
    void MarginalizeGlobalInfo();

    //Only for ceres
    void SolveFGOCeres();
    void MarginalizeCeres();


    //TemplateSolver
    void SolveBATemplate();
    void MarginalizeBATemplate();

    void double2data();
    void SlidingWindow();
    void SlidingWindowOld();
    void RemoveAllImageFrame();
    void RemoveAllNegativeDpeth();
    Sophus::SO3d gravity2Rotation(Eigen::Vector3d &gravity);

    // bool CheckMotion(GNSSFramePtr first_obs_states, GNSSFramePtr
    // end_obs_states); bool CheckZUPT();

    // slam param
    ParamReaderPtr gnss_parameter;
    State state;
    double focal_length;
    double gyr_n, acc_n, gyr_w, acc_w;
    Eigen::Matrix3d gyr_noise_cov;
    Eigen::Matrix3d acc_noise_cov;
    Eigen::Matrix<double, 6, 6> gyr_acc_noise_cov;
    Eigen::Matrix3d gyr_bias_cov;
    Eigen::Matrix3d acc_bias_cov;
    Eigen::Matrix<double, 6, 6> acc_gyr_bias_invcov;
    double gnss_imu_time_delay = 0.0; // time offset between imu and cam
    int gnss_window_size = 20;
    std::deque<GNSSFramePtr> d_gnss_frames; // [ 0,  1, ..., 8 ,         9 |  30] size 30
    //  kf  kf      kf  second new   new
    MarginTypeGNSS fgo_marginal_flag;

    // vsolver data
    double *para_pose;  // Twb
    double *para_speed; // vwb
    double *para_bias;  // ba / bg

    //only for ceres
    // double *para_speed_bias; //only for ceres
    std::vector<double *> last_marginalization_info_para_block;
    std::shared_ptr<MarginalizationInfo> last_margin_info;

    // maintain system, system.cpp backend Reset --> backend->ResetRequest() -->
    // using backend inline function set request_reset_flag true
    std::atomic<bool> request_reset_flag; // not use

    // Vsolver Prior_info
    MatXX Hprior_;
    VecX bprior_;
    VecX errprior_;
    MatXX Jprior_inv_;
    MatXX Jprior_;

    // TemplateSolverMember
    GraphOptimizor::MatrixX<Scalar> prior_H, prior_JTinv;
    GraphOptimizor::VectorX<Scalar> prior_b, prior_r;

    // GNSS-Imu init, only for esimate Rwb_init
    //計算初始平均標準差與平均, 透過標準差絕對是否穩定,
    //透過平均找到垂直重力的初始與世界的旋轉差
    bool initGPS = false;
    std::deque<double> zupt_acc_norm;                                         // gyro_norm, acc_norm, acc_xyz
    double auto_detect_imu_fps = -1;
    double gnss_system_initial_timestamp;
    Eigen::Vector3d station_origin_llh;
    Eigen::Vector3d tmp_lever_arm;
    double cv_huber_loss_parameter = 1;
};

using FGO_GpsIMU_SOLVERPtr = std::shared_ptr<FGO_GpsIMU_SOLVER>;
using FGO_GpsIMU_SOLVERConstPtr = std::shared_ptr<const FGO_GpsIMU_SOLVER>;
