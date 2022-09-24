/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ob_gins/common/earth.h"
#include "ob_gins/common/types.h"

#include "ob_gins/fileio/filesaver.h"
#include "ob_gins/fileio/gnssfileloader.h"
#include "ob_gins/fileio/imufileloader.h"

#include "ob_gins/factors/gnss_factor.h"
#include "ob_gins/factors/marginalization_factor.h"
#include "ob_gins/factors/pose_parameterization.h"
#include "ob_gins/preintegration/imu_error_factor.h"
#include "ob_gins/preintegration/preintegration.h"
#include "ob_gins/preintegration/preintegration_factor.h"
#include <thread>
#include <chrono>

#include <absl/time/clock.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>

//our solver
#include <chrono>
#include <thread>
#include <slam_vio/sophus_extra.h>

// slam parameter
#include "slam_vio/integration_base.h"
#include "slam_vio/bull_integration_base.h"
#include "start_slam_sys/slam_param_reader.h"
#include "fgo_gps_imu/fgo_gps_imu_solver.h"
// our V-solver
#include "slam_vio/Vsolver/edge_imu.h"
#include "slam_vio/Vsolver/edge_gnss.h"
#include "slam_vio/Vsolver/problem.h"
#include "slam_vio/Vsolver/vertex_bias.h"
#include "slam_vio/Vsolver/vertex_pose.h"
#include "slam_vio/Vsolver/vertex_speed.h"
#include "slam_vio/Vsolver/vertex_speedbias.h"
#include "slam_vio/Vsolver/vertex_td.h"

#define ENABLE_OB_GINS 1
#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001
#define ROS_OUTPUT 1

#if ROS_OUTPUT
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <message_filters/time_synchronizer.h>
static ros::Publisher pub_fuse_path, pub_vsolver_fuse_path; // static --> only this cpp can read
static ros::Publisher pub_gnss_only_path;                   // are vio pose.
static ros::Publisher pub_groundtruth_path;                 // are vio pose.

static ros::Publisher pub_slidingwindow_gnss_only_path; // are vio pose.
static ros::Publisher pub_slidingwindow_fuse_path;      // are vio pose.

static nav_msgs::Path slidingwindow_fuse_path;
static nav_msgs::Path slidingwindow_gnss_only_path;
static nav_msgs::Path fuse_path;
static nav_msgs::Path gnss_only_path;
static nav_msgs::Path vsolver_fuse_path;
static nav_msgs::Path groundtruth_path;
using namespace ros;
using namespace message_filters;
using namespace sensor_msgs;
#endif

FGO_GpsIMU_SOLVERPtr fgo_vsolver;
static int imu_integration_count = 0;
static int ceres_solver_count = 0;


std::string readFileIntoString(const std::string &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open())
    {
        std::cout << "Could not open the file - '"
                  << path << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}

void readCsvData(const std::string &input_csv_name_pose,
                 std::map<int, std::vector<std::string>> &csv_result,
                 char delimiter)
{
    //read pose csv data.
    std::string file_contents_pose = readFileIntoString(input_csv_name_pose);
    std::istringstream sstream(file_contents_pose);
    std::vector<std::string> items;
    std::string record;
    int counter = 0;
    while (std::getline(sstream, record))
    {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter))
        {
            record.erase(std::remove_if(record.begin(), record.end(), isspace), record.end());
            items.push_back(record);
        }
        csv_result[counter] = items;
        items.clear();
        counter += 1;
    }
}




int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);

int main(int argc, char *argv[])
{

#if ROS_OUTPUT
    std::cout << "\033[1;31mbold GNSS_IMU_FGO ready \033[0m\n"
              << std::endl;
    init(argc, argv, "gnss_imu_node", init_options::NoSigintHandler);
    NodeHandle nh("~");
    pub_fuse_path = nh.advertise<nav_msgs::Path>("fuse_obgins_path", 10);
    pub_vsolver_fuse_path = nh.advertise<nav_msgs::Path>("fuse_vsolver_path", 10);
    pub_gnss_only_path = nh.advertise<nav_msgs::Path>("gnss_only_path", 10);
    pub_groundtruth_path = nh.advertise<nav_msgs::Path>("GT_path", 10);
    pub_slidingwindow_gnss_only_path = nh.advertise<nav_msgs::Path>("sliding_window_gnss_path", 10);
    pub_slidingwindow_fuse_path = nh.advertise<nav_msgs::Path>("sliding_window_fuse_path", 10);
#endif

    if (argc != 2)
    {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::string fuse_result_save_path = std::string("ob_fuse_path.csv");
    std::string gnss_only_save_path = std::string("ob_gnss_path.csv");
    std::string groundtrue_save_path = std::string("gt_path.csv");
    std::ofstream fout(fuse_result_save_path);
    std::ofstream fout1(gnss_only_save_path);
    std::ofstream fout2(groundtrue_save_path);
    auto ts = absl::Now();

    // 读取配置
    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try
    {
        config = YAML::LoadFile(argv[1]);
    }
    catch (YAML::Exception &exception)
    {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // 时间信息
    // processing time
    int windows = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime = config["endtime"].as<int>();

    // 初始化信息
    // initialization
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // 数据文件
    // data file
    std::string gnsspath = config["gnssfile"].as<std::string>();
    std::string imupath = config["imufile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();
    int imudatalen = config["imudatalen"].as<int>();
    int imudatarate = config["imudatarate"].as<int>();

    // 是否考虑地球自转
    // consider the Earth's rotation
    bool isearth = config["isearth"].as<bool>();

    //input imu-gnss
    GnssFileLoader gnssfile(gnsspath);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);

    std::cout << "gnsspath=" << gnsspath << std::endl;
    std::cout << "imufile=" << imupath << std::endl;

    //output nav_result
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen())
    {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }


    // 安装参数
    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;

    // IMU噪声参数
    // IMU noise parameters
    auto parameters = std::make_shared<IntegrationParameters>();
    parameters->gyr_arw = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    parameters->acc_vrw = config["imumodel"]["vrw"].as<double>() / 60.0;
    parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    parameters->corr_time = config["imumodel"]["corrtime"].as<double>() * 3600;

    bool isuseodo = config["odometer"]["isuseodo"].as<bool>();
    vec = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo = odolever;
    parameters->abv = bodyangle;

    // GNSS仿真中断配置
    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime = config["outagetime"].as<int>();
    int outagelen = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    std::cout << "use odom=" << isuseodo << std::endl;
    std::cout << "use earth_rotation=" << isearth << std::endl;
    std::cout << "use isuseoutage=" << isuseoutage << std::endl;

    // 数据文件调整
    // data alignment (make imu and gnss factor timestamp init_timestamp >= starttime)
    std::cout << "starttime=" << starttime << std::endl;
    IMU imu_cur, imu_pre;
    do
    {
        imu_pre = imu_cur;
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);
    GNSS gnss;
    do
    {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    Matrix3d ned2enu;
    ned2enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;

    // 初始位置, 求相对
    Vector3d station_origin = gnss.blh;
    parameters->gravity = Earth::gravity(gnss.blh);
    std::cout << "init gnss.blh = " << gnss.blh << std::endl;
    //output is NED coordinate.
    gnss.blh = Earth::global2local(station_origin, gnss.blh);
    gnss.blh = ned2enu * gnss.blh;


    //convert ground Truth result to fout2
    //delimiter sign
    #if 0 
    char delimiter = ',';
    char delimiter_space = ' ';
    std::map<int, std::vector<std::string>> gt_path_result_llh;
    readCsvData("/datasets/learn_gins_ws/src/navgation_module/config/ob_dataset/truth.txt", gt_path_result_llh, delimiter);
    std::cout << "gt_path_result_llh.size()=" << gt_path_result_llh.size() <<std::endl;
    
    for (int i = 0, n = gt_path_result_llh.size(); i < n; i++)
    {
        //format: tx ty heading noise_tx noise_ty noise_heading(radian)
        Eigen::Matrix<double, 7, 1> pose_units;
        double timestamp = std::atof(gt_path_result_llh[i][1].c_str()); //sec
        Eigen::Vector3d llh_gt, ned_xyz_gt;
        llh_gt.x() = std::atof(gt_path_result_llh[i][2].c_str()) * D2R; //latitude latitude degree
        llh_gt.y() = std::atof(gt_path_result_llh[i][3].c_str()) * D2R; //longitude latitude degree
        llh_gt.z() = std::atof(gt_path_result_llh[i][4].c_str());       //height latitude degree

        ned_xyz_gt = Earth::global2local(station_origin, llh_gt);
        Eigen::Vector3d ea0(std::atof(gt_path_result_llh[i][10].c_str()),std::atof(gt_path_result_llh[i][9].c_str()),std::atof(gt_path_result_llh[i][8].c_str()));
        Eigen::Matrix3d R_ned_xyz;
        R_ned_xyz = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_ned_xyz;
        q_ned_xyz = R_ned_xyz;

        Eigen::Matrix3d Renu_ned;
        Renu_ned<< 0,1,0,1,0,0,0,0,-1;
// 
        Eigen::Vector3d enu_xyz_gt;
        enu_xyz_gt.x() = ned_xyz_gt.y();
        enu_xyz_gt.y() = ned_xyz_gt.x();
        enu_xyz_gt.z() = -ned_xyz_gt.z();

        Eigen::Quaterniond q_enu_xyz((Renu_ned * R_ned_xyz).normalized());
        std::cout << timestamp <<std::endl;
        fout2 << std::setprecision(20) << timestamp << " " 
                                       << enu_xyz_gt.x() << " "
                                       << enu_xyz_gt.y() << " "
                                       << enu_xyz_gt.z() << " "
                                       << q_enu_xyz.x() << " "
                                       << q_enu_xyz.y() << " "
                                       << q_enu_xyz.z() << " "
                                       << q_enu_xyz.w() << std::endl;

        
    }  
    #endif

    // 站心坐标系原点
    parameters->station = station_origin;
    std::vector<std::shared_ptr<PreintegrationBase>> preintegrationlist;
    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::vector<GNSS> gnsslist;
    std::vector<double> timelist;
    Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(isuseodo, isearth);
    std::cout << "initatt=" << initatt << std::endl;
    std::cout << "antlever=" << antlever << std::endl;

// 初始状态
// initialization
#if 0
    IntegrationState state_curr = {
        .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = initvel,
        .bg   = initbg,
        .ba   = initba,
        .sodo = 0.0,
        .abv  = {bodyangle[1], bodyangle[2]},
    };
#else
    // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp
    // Three axises of the ENU frame in the IMU frame.
    // z-axis.
    const Eigen::Vector3d &z_axis = ((imu_pre.dvel) * 200).normalized();
    // x-axis.
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();
    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;
    Eigen::Quaterniond tmp_Rwb(I_R_G.transpose());
    tmp_Rwb.normalize();

    IntegrationState state_curr;
    state_curr.p = gnss.blh - Rotation::euler2quaternion(initatt) * antlever;
    // state_curr.q = Rotation::euler2quaternion(initatt);
    state_curr.q = tmp_Rwb;
    state_curr.v = initvel;
    state_curr.bg = initbg;
    state_curr.ba = initba;
    state_curr.sodo = 0.0;
    state_curr.abv = {bodyangle[1], bodyangle[2]};
#endif

    std::cout << "Initilization at " << gnss.time << " s" << std::endl;
    std::cout << "initatt=" << initatt << std::endl;
    std::cout << "antlever=" << antlever << std::endl;
    std::cout << "state_curr.p : " << state_curr.p << std::endl;
    std::cout << "state_curr.q : " << state_curr.q.coeffs() << std::endl;
    std::cout << "state_curr.v : " << state_curr.v << std::endl;
    std::cout << "state_curr.bg : " << state_curr.bg << std::endl;
    std::cout << "state_curr.ba : " << state_curr.ba << std::endl;
    std::cout << "state_curr.sodo : " << state_curr.sodo << std::endl;
    std::cout << "state_curr.abv : " << state_curr.abv << std::endl;

    statelist[0] = state_curr;
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // 初始预积分
    // Initial preintegration
    preintegrationlist.emplace_back(Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));

    //Step1. InitSystem
    //Vsolver
    double imu_fps = 1.0 / (imu_cur.time - imu_pre.time);

    //VsolverInit, est first Rwb from imu to ENU world
    fgo_vsolver = std::make_shared<FGO_GpsIMU_SOLVER>(parameters->gyr_arw, parameters->acc_vrw, parameters->gyr_bias_std, parameters->acc_bias_std, windows);
    FGO_GpsIMU_SOLVER::GNSSFramePtr cur_frame = std::make_shared<FGO_GpsIMU_SOLVER::GNSSFrame>();
    cur_frame->timestamp = timelist.back();
    {
        Eigen::Vector3d convert_acc = imu_pre.dvel * 200;
        Eigen::Vector3d convert_gyro = imu_pre.dtheta * 200;
        cur_frame->v_imu_timestamp.emplace_back(imu_pre.time);
        cur_frame->v_acc.emplace_back(convert_acc);
        cur_frame->v_gyr.emplace_back(convert_gyro);
    }
    cur_frame->ori_blh = station_origin;
    cur_frame->gnss_llh_variance = gnsslist.back().std;
    fgo_vsolver->ProcessGNSSIMU(cur_frame);

    // 读取下一个整秒GNSS
    cur_frame = std::make_shared<FGO_GpsIMU_SOLVER::GNSSFrame>();
    gnss = gnssfile.next();
    GNSS last_gnss_blh_ori = gnss;
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh = Earth::global2local(station_origin, gnss.blh);
    gnss.blh = ned2enu * gnss.blh;

    // 边缘化信息
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

    // 下一个积分节点
    sow += INTEGRATION_LENGTH;
    cur_frame->timestamp = sow;
    while (true)
    {
        if ((imu_cur.time > endtime) || imufile.isEof())
        {
            break;
        }

        // 加入IMU数据
        // Add new imu data to preintegration
        {
// std::cout << "==========IMU_OB intergration===============" << imu_integration_count << std::endl;
#if ENABLE_OB_GINS
            preintegrationlist.back()->addNewImu(imu_cur);
#endif

            //[Vsolver]  step2. Vsolver latest frame integration.
            imu_fps = 1.0 / (imu_cur.time - imu_pre.time);
            Eigen::Vector3d acc_cur = imu_cur.dvel * imu_fps;
            Eigen::Vector3d gyro_cur = imu_cur.dtheta * imu_fps;
            cur_frame->v_acc.push_back(acc_cur);
            cur_frame->v_gyr.push_back(gyro_cur);
            cur_frame->v_imu_timestamp.push_back(imu_cur.time);
            imu_integration_count++;
        }

        imu_pre = imu_cur;
        imu_cur = imufile.next();
        Eigen::Vector3d cur_gnss_local_pose;
        double fuse_frame_timestamp = ros::Time::now().toSec();
        if (imu_cur.time > sow)
        {
            // 当前IMU数据时间等于GNSS数据时间, 读取新的GNSS
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL)
            {
                gnsslist.push_back(gnss);

                gnss = gnssfile.next();
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold))
                {
                    gnss = gnssfile.next();
                }

                // 中断配置
                // do GNSS outage
                if (isuseoutage)
                {
                    if (lround(gnss.time) == outagetime)
                    {
                        std::cout << "GNSS outage at " << outagetime << " s" << std::endl;
                        for (int k = 0; k < outagelen; k++)
                        {
                            gnss = gnssfile.next();
                        }
                        outagetime += outageperiod;
                    }
                }

                // [Vsolver]  step3. add GNSS factor info to correspondence keyframe.
                cur_frame->gnss_llh_variance = last_gnss_blh_ori.std;
                cur_frame->ori_blh = last_gnss_blh_ori.blh;

                if (fabs(cur_frame->timestamp - last_gnss_blh_ori.time) >= MINIMUM_INTERVAL)
                {
                    std::cout << "not align frame and gnss time" << std::endl;
                    exit(-1);
                }

                //GPS not refine gravity
                // parameters->gravity = Earth::gravity(gnss.blh);
                last_gnss_blh_ori = gnss;
                gnss.blh = Earth::global2local(station_origin, gnss.blh);
                gnss.blh = ned2enu * gnss.blh;
                cur_gnss_local_pose = gnss.blh;

                if (gnssfile.isEof())
                {
                    gnss.time = 0;
                }
            }

// IMU内插处理
// IMU interpolation
#if 0
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1)
            {
            }
            else if (isneed == 1)
            {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = imufile.next();
            }
            else if (isneed == 2)
            {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }
#endif

            // 下一个积分节点
            // next time node
            timelist.push_back(sow);
            sow += INTEGRATION_LENGTH;

            //[Vsolver]  step4. push collected imu and gnss info to cur_frame and push to fgo system.
            std::cout << "@==Vsovler FACTOR ==@" << std::endl;
            FGO_GpsIMU_SOLVER::GNSSResult vsolver_res = fgo_vsolver->ProcessGNSSIMU(cur_frame);

#if !ENABLE_OB_GINS
            fout << std::setprecision(20) << vsolver_res.cur_frame_result->timestamp << " ";
            fout << vsolver_res.cur_frame_result->p_wb.x() << " "
                 << vsolver_res.cur_frame_result->p_wb.y() << " "
                 << vsolver_res.cur_frame_result->p_wb.z() << " "
                 << vsolver_res.cur_frame_result->q_wb.unit_quaternion().x() << " "
                 << vsolver_res.cur_frame_result->q_wb.unit_quaternion().y() << " "
                 << vsolver_res.cur_frame_result->q_wb.unit_quaternion().z() << " "
                 << vsolver_res.cur_frame_result->q_wb.unit_quaternion().w() << std::endl;
#endif
            //[Vsolver]  step5. REnew cur_frame for collect next sec imu and gnss data.
            cur_frame = std::make_shared<FGO_GpsIMU_SOLVER::GNSSFrame>();
            cur_frame->timestamp = sow;

            // 当前整秒状态加入到滑窗中
            //
            state_curr = preintegrationlist.back()->currentState();
            statelist[preintegrationlist.size()] = state_curr;
            statedatalist[preintegrationlist.size()] = Preintegration::stateToData(state_curr, preintegration_options);

            // 构建优化问题
            // construct optimization problem
            {
#if ENABLE_OB_GINS
                std::cout << "@==OBGINS FACTOR ==@" << std::endl;
                ceres_solver_count++;
                ceres::Solver solver;
                ceres::Problem problem;
                ceres::Solver::Summary summary;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.trust_region_strategy_type = ceres::DOGLEG;
                // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                options.num_threads = 1;
                options.max_num_iterations = 10;

                // 参数块
                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++)
                {
                    // 位姿
                    // std::cout << "statedatalist[k].pose=" << statedatalist[k].pose << std::endl;
                    ceres::LocalParameterization *parameterization = new (PoseParameterization);
                    problem.AddParameterBlock(statedatalist[k].pose, Preintegration::numPoseParameter(),
                                              parameterization);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Preintegration::numMixParameter(preintegration_options));

                    // #if ROS_OUTPUT
                    //                     {
                    //                         IntegrationState tmp = statelist[k];
                    //                         // std::cout << "preintegrationlist.size():" << preintegrationlist.size() << std::endl;
                    //                         // std::cout << "k=" << k << " frame time=" << std::setprecision(20) << tmp.time << std::endl;
                    //                         // Pub Path
                    //                         double fuse_frame_timestamp = ros::Time::now().toSec();
                    //                         // slidingwindow_fuse_path.poses.clear();
                    //                         slidingwindow_fuse_path.header.seq = 0;
                    //                         slidingwindow_fuse_path.header.stamp.fromSec(fuse_frame_timestamp);
                    //                         slidingwindow_fuse_path.header.frame_id = "world";
                    //                         geometry_msgs::PoseStamped pose_stamped;
                    //                         pose_stamped.header.stamp.fromSec(fuse_frame_timestamp);
                    //                         pose_stamped.header.frame_id = "sliding_window_fuse_path";

                    //                         pose_stamped.pose.position.x = tmp.p[0];
                    //                         pose_stamped.pose.position.y = tmp.p[1];
                    //                         pose_stamped.pose.position.z = tmp.p[2];
                    //                         pose_stamped.pose.orientation.w = tmp.q.w();
                    //                         pose_stamped.pose.orientation.x = tmp.q.x();
                    //                         pose_stamped.pose.orientation.y = tmp.q.y();
                    //                         pose_stamped.pose.orientation.z = tmp.q.z();
                    //                         slidingwindow_fuse_path.poses.emplace_back(pose_stamped);
                    //                         pub_slidingwindow_fuse_path.publish(slidingwindow_fuse_path);
                    //                         std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    //                     }
                    // #endif
                }

                // std::cout << "preintegrationlist.size()=" << preintegrationlist.size() <<std::endl;
                // std::cout << "gnsslist.size()=" << gnsslist.size() <<std::endl;

                // GNSS残差
                // GNSS factors
                // Add outlier culling as you need
                int index = 0;
                for (auto &data : gnsslist)
                {
                    auto factor = new GnssFactor(data, antlever);

                    // std::cout << "data.pose=" << data << std::endl;
                    for (size_t i = index; i <= preintegrationlist.size(); ++i)
                    {
                        if (fabs(data.time - timelist[i]) < MINIMUM_INTERVAL)
                        {
#if ROS_OUTPUT
                            {
                                // std::cout << "index=" << index << " gnss time=" << std::setprecision(20) << data.time << std::endl;
                                // Pub Path
                                double fuse_frame_timestamp = ros::Time::now().toSec();
                                // slidingwindow_gnss_only_path.poses.clear();
                                slidingwindow_gnss_only_path.header.seq = 0;
                                slidingwindow_gnss_only_path.header.stamp.fromSec(fuse_frame_timestamp);
                                slidingwindow_gnss_only_path.header.frame_id = "world";
                                geometry_msgs::PoseStamped pose_stamped;
                                pose_stamped.header.stamp.fromSec(fuse_frame_timestamp);
                                pose_stamped.header.frame_id = "sliding_window_gnss_path";
                                pose_stamped.pose.position.x = data.blh[0];
                                pose_stamped.pose.position.y = data.blh[1];
                                pose_stamped.pose.position.z = data.blh[2];
                                slidingwindow_gnss_only_path.poses.emplace_back(pose_stamped);
                                pub_slidingwindow_gnss_only_path.publish(slidingwindow_gnss_only_path);
                                // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            }
#endif
                            problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);
                            std::cout << "OBGINS GNSS data=" << data.time << std::endl;
                            std::cout << "OBGINS GNSS blh=" << data.blh << std::endl;
                            index++;
                            break;
                        }
                    }
                }

                // 预积分残差
                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++)
                {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    std::cout << "===" << std::endl;
                    std::cout << "OBGINS preintegrationlist delta_t=" << preintegrationlist[k]->endTime() << std::endl;
                    std::cout << "OBGINS preintegrationlist delta_p=" << preintegrationlist[k]->deltaState().p << std::endl;
                    std::cout << "OBGINS preintegrationlist delta_v=" << preintegrationlist[k]->deltaState().v << std::endl;
                    std::cout << "OBGINS preintegrationlist delta_q=" << preintegrationlist[k]->deltaState().q.coeffs() << std::endl;
                    std::cout << "OBGINS preintegrationlist sumT=" << preintegrationlist[k]->deltaTime() << std::endl;
                    std::cout << "OBGINS preintegrationlist currState_p=" << preintegrationlist[k]->currentState().p << std::endl;
                    std::cout << "OBGINS preintegrationlist currState_v=" << preintegrationlist[k]->currentState().v << std::endl;
                    std::cout << "OBGINS preintegrationlist currState_q=" << preintegrationlist[k]->currentState().q.coeffs() << std::endl;
                    std::cout << "OBGINS preintegrationlist ba=" << preintegrationlist[k]->currentState().ba << std::endl;
                    std::cout << "OBGINS preintegrationlist bg=" << preintegrationlist[k]->currentState().bg << std::endl;
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                
                {
                    // IMU误差控制
                    // add IMU bias-constraint factors
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // 边缘化残差
                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid())
                {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                // 求解最小二乘
                // solve the Least-Squares problem
                solver.Solve(options, &problem, &summary);
                //                std::cout << sow - 1 << ": " << summary.BriefReport() << std::endl;

                // 输出进度
                // output the percentage
                int percent = ((int)sow - starttime) * 100 / (endtime - starttime);
                static int lastpercent = 0;
                if (abs(percent - lastpercent) >= 1)
                {
                    lastpercent = percent;
                    std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                    flush(std::cout);
                }
#endif
            }

            std::cout << "============Solved Result===========" <<std::endl;
            std::cout << "vsolver_res times=" << vsolver_res.cur_frame_result->timestamp <<std::endl;
            std::cout << "OB_GINS_res times=" << statedatalist[preintegrationlist.size()].time <<std::endl;
            for (int p_idx = 0; p_idx < 7; p_idx++)
            {
                std::cout << "OB_GINS_res p_wb=" << statedatalist[preintegrationlist.size()].pose[p_idx] << std::endl;
            }
            for (int p_idx = 0; p_idx < 9; p_idx++)
            {
                std::cout << "OB_GINS_res mix=" << statedatalist[preintegrationlist.size()].mix[p_idx] << std::endl;
            }

            std::cout << "vsolver_res p_wb=" << vsolver_res.cur_frame_result->p_wb << std::endl;
            std::cout << "vsolver_res v_wb=" << vsolver_res.cur_frame_result->v_wb << std::endl;
            std::cout << "vsolver_res q_wb=" << vsolver_res.cur_frame_result->q_wb.unit_quaternion().coeffs() << std::endl;
            std::cout << "vsolver_res ba=" << vsolver_res.cur_frame_result->ba << std::endl;
            std::cout << "vsolver_res bg=" << vsolver_res.cur_frame_result->bg << std::endl;

//draw VsolverResultPose.
#if ROS_OUTPUT
            {
                // Pub Path
                vsolver_fuse_path.header.seq = 0;
                vsolver_fuse_path.header.stamp.fromSec(fuse_frame_timestamp);
                vsolver_fuse_path.header.frame_id = "world";
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp.fromSec(fuse_frame_timestamp);
                pose_stamped.header.frame_id = "fuse_fgo";
                IntegrationState tmp = preintegrationlist.back()->currentState();
                pose_stamped.pose.position.x = vsolver_res.cur_frame_result->p_wb[0];
                pose_stamped.pose.position.y = vsolver_res.cur_frame_result->p_wb[1];
                pose_stamped.pose.position.z = vsolver_res.cur_frame_result->p_wb[2];
                pose_stamped.pose.orientation.w = vsolver_res.cur_frame_result->q_wb.unit_quaternion().w();
                pose_stamped.pose.orientation.x = vsolver_res.cur_frame_result->q_wb.unit_quaternion().x();
                pose_stamped.pose.orientation.y = vsolver_res.cur_frame_result->q_wb.unit_quaternion().y();
                pose_stamped.pose.orientation.z = vsolver_res.cur_frame_result->q_wb.unit_quaternion().z();
                vsolver_fuse_path.poses.emplace_back(pose_stamped);
                pub_vsolver_fuse_path.publish(vsolver_fuse_path);
            }
#endif

//draw OB_GINS Result Pose.
#if ROS_OUTPUT
            {
                // Pub Path
                fuse_path.header.seq = 0;
                fuse_path.header.stamp.fromSec(fuse_frame_timestamp);
                fuse_path.header.frame_id = "world";
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp.fromSec(fuse_frame_timestamp);
                pose_stamped.header.frame_id = "fuse_obgins";
                IntegrationState tmp = preintegrationlist.back()->currentState();
                pose_stamped.pose.position.x = statedatalist[preintegrationlist.size()].pose[0];
                pose_stamped.pose.position.y = statedatalist[preintegrationlist.size()].pose[1];
                pose_stamped.pose.position.z = statedatalist[preintegrationlist.size()].pose[2];
                pose_stamped.pose.orientation.w = statedatalist[preintegrationlist.size()].pose[6]; //w
                pose_stamped.pose.orientation.x = statedatalist[preintegrationlist.size()].pose[3]; //x
                pose_stamped.pose.orientation.y = statedatalist[preintegrationlist.size()].pose[4]; //y
                pose_stamped.pose.orientation.z = statedatalist[preintegrationlist.size()].pose[5]; //z
                fuse_path.poses.emplace_back(pose_stamped);
                pub_fuse_path.publish(fuse_path);
            }
#endif
            // if(ceres_solver_count == 2)
            //     exit(-1);

            if (preintegrationlist.size() == static_cast<size_t>(windows))
            {
                // exit(-1);
                {
#if ENABLE_OB_GINS
                    // 边缘化
                    // marginalization
                    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                    if (last_marginalization_info && last_marginalization_info->isValid())
                    {

                        std::vector<int> marginilized_index;
                        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++)
                        {
                            if (last_marginalization_parameter_blocks[k] == statedatalist[0].pose ||
                                last_marginalization_parameter_blocks[k] == statedatalist[0].mix)
                            {
                                marginilized_index.push_back(static_cast<int>(k));
                            }
                        }


                        auto factor = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // IMU残差
                    // preintegration factors
                    {
                        auto factor = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS残差
                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL)
                        {
                            auto factor = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    // 边缘化处理
                    // do marginalization
                    marginalization_info->marginalization();

                    // 数据指针调整
                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++)
                    {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)] = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info = std::move(marginalization_info);
#endif
                }

                // 滑窗处理
                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time))
                    {
                        gnsslist.erase(gnsslist.begin());
                    }
                    timelist.erase(timelist.begin());
                    preintegrationlist.erase(preintegrationlist.begin());

                    for (int k = 0; k < windows; k++)
                    {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k] = Preintegration::stateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Preintegration::stateFromData(statedatalist[windows], preintegration_options);
                    state_curr = statelist[windows];
                }
            }
            else
            {
                state_curr = Preintegration::stateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // write result
            writeNavResult(*timelist.rbegin(), station_origin, state_curr, navfile, errfile);

    

#if ENABLE_OB_GINS
            fout << std::setprecision(20) << *timelist.rbegin() << " ";
            fout << state_curr.p.x() << " "
                 << state_curr.p.y() << " "
                 << state_curr.p.z() << " "
                 << state_curr.q.x() << " "
                 << state_curr.q.y() << " "
                 << state_curr.q.z() << " "
                 << state_curr.q.w() << std::endl;

            fout1 << std::setprecision(20) << *timelist.rbegin() << " ";
            fout1 << cur_gnss_local_pose.x() << " "
                  << cur_gnss_local_pose.y() << " "
                  << cur_gnss_local_pose.z() << " "
                  << 0.0 << " "
                  << 0.0 << " "
                  << 0.0 << " "
                  << 1.0 << std::endl;
#endif

            // 新建立新的预积分
            // build a new preintegration object
            preintegrationlist.emplace_back(
                Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
        }
        else
        {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    gnssfile.close();

    auto te = absl::Now();
    std::cout << "\r\nCost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;
    std::cout << "antlever=" << antlever << std::endl;
    return 0;
}

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile)
{
    static int counts = 0;
    if ((counts++ % 10) != 0)
    {
        return;
    }

    vector<double> result;

    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg = state.bg * R2D * 3600;
    Vector3d ba = state.ba * 1e5;

    {
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid)
{
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff = imu01;

    imu00.time = time;
    imu00.dt = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);

    imu11.time = buff.time;
    imu11.dt = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid)
{
    double time = mid;

    if (imu0.time < time && imu1.time > time)
    {
        double dt = time - imu0.time;

        // 前一个历元接近
        // close to the first epoch
        if (dt < 0.0001)
        {
            return -1;
        }

        // 后一个历元接近
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001)
        {
            return 1;
        }

        // 需内插
        // need interpolation
        return 2;
    }

    return 0;
}
