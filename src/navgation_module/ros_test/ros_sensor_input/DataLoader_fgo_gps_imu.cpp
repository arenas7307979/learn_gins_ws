// catch ctrl+c signal
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include "signal.h"
#include "fgo_gps_imu/fgo_gps_imu_solver.h"

// rosrun fgo_imu_gnss fgo_gps_imu_node src/navgation_module/config/imu_gps_fuse_datasets/mobile_one_for_all_withgps.yaml /slam_dataset_mac/iphone_datasets/2022-09-15T21-47-46/SyncAccGyro.txt  /slam_dataset_mac/iphone_datasets/2022-09-15T21-47-46/GPS.txt

#define OB_GINS 1
#define ROS_OUTPUT 1
#define SLAM_BACKEND_DEBUG_LOG 0


#if ROS_OUTPUT
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <message_filters/time_synchronizer.h>
static ros::Publisher pub_fuse_path;                    // static --> only this cpp can read
static ros::Publisher pub_gnss_only_path; // are vio pose.
static ros::Publisher pub_groundtruth_path; // are vio pose.
static nav_msgs::Path fuse_path;
static nav_msgs::Path gnss_only_path;
static nav_msgs::Path groundtruth_path;
using namespace ros; 
using namespace message_filters;
using namespace sensor_msgs;
#endif

std::string readFileIntoString(const std::string &path) {
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        std::cout << "Could not open the file - '" << path << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}

void readCsvData(const std::string &input_csv_name_pose,
                 std::map<int, std::vector<std::string>> &csv_result,
                 char delimiter) {
    // read pose csv data.
    std::string file_contents_pose = readFileIntoString(input_csv_name_pose);
    std::istringstream sstream(file_contents_pose);
    std::vector<std::string> items;
    std::string record;
    int counter = 0;
    while (std::getline(sstream, record)) {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter)) {
            record.erase(std::remove_if(record.begin(), record.end(), isspace),
                         record.end());
            items.push_back(record);
        }
        csv_result[counter] = items;
        items.clear();
        counter += 1;
    }
}

// under path of /slam_core
// example::
// rosrun slam_corevis fgo_gps_imu_node
// src/navgation_module/config/imu_gps_fuse_datasets/mobile_one_for_all_withgps.yaml
// src/navgation_module/config/imu_gps_fuse_datasets/ICM20602.txt
// src/navgation_module/config/imu_gps_fuse_datasets/GNSS_RTK.pos
// src/navgation_module/config/imu_gps_fuse_datasets/truth.nav

//global parameter ----------
// NOTE:
//由於ob_gins輸入的imu_raw數據單位, 相較於傳統imu少除上s,
//因此前面補上imu_hz數目 * 200 一般而言, 傳統imu數據, 補償係數設定為1
int ignore_gps_number = 10;
int time_to_ignore_gps_number = 35;

int main(int argc, char **argv) {
#if ROS_OUTPUT
    std::cout << "\033[1;31mbold GNSS_IMU_FGO ready \033[0m\n"
              << std::endl;
    init(argc, argv, "gnss_imu_node", init_options::NoSigintHandler);
    NodeHandle nh("~");
    pub_fuse_path = nh.advertise<nav_msgs::Path>("fuse_vsolver_path", 10);
    pub_gnss_only_path = nh.advertise<nav_msgs::Path>("gnss_only_path", 10);
    pub_groundtruth_path = nh.advertise<nav_msgs::Path>("GT_path", 10);
#endif

    if (argc < 4) {
        std::cout << "Input path not enough < 4 [input_sensors_yaml_path] "
                     "[imu_data_path] [gnss_data_path]"
                  << std::endl;
    }
    // delimiter sign
    char delimiter = ',';
    char delimiter_space = ' ';

    std::string input_sensors_yaml_path = std::string(argv[1]);
    std::string imu_data_path = std::string(argv[2]);
    std::string gnss_data_path = std::string(argv[3]);
    std::cout << "imu_data_path=" << imu_data_path << std::endl;
    std::cout << "gnss_data_path=" << gnss_data_path << std::endl;
    std::map<int, std::vector<std::string>> csv_imu_raw, csv_gps_info;

    // InitSystemParaFromYaml
    ParamReaderPtr parameter_reader = std::make_shared<ParamReader>(input_sensors_yaml_path);
    FGO_GpsIMU_SOLVERPtr fgo_vsolver = std::make_shared<FGO_GpsIMU_SOLVER>(parameter_reader);

    readCsvData(imu_data_path, csv_imu_raw, delimiter);
    readCsvData(gnss_data_path, csv_gps_info, delimiter);
    //time(s), acc, gyro
    std::vector<Eigen::Matrix<double, 7, 1>> imu_raw_units_vec;
    // time(s),latitude(deg),longitude(deg),horizontalAccuracy(m),altitude(m),verticalAccuracy(m),floorLevel,course(dgr),speed(m/s)
    std::vector<Eigen::Matrix<double, 10, 1>> gps_units_vec;


    for (int i = 0, n = csv_gps_info.size(); i < n; i++) {
        Eigen::Matrix<double, 10, 1> gps_unit;
        gps_unit[0] = std::atof(csv_gps_info[i][0].c_str()); // GNSS second
        gps_unit[1] = std::atof(csv_gps_info[i][1].c_str()) * parameter_reader->D2R; //geodetic latitude deg to radius
        gps_unit[2] = std::atof(csv_gps_info[i][2].c_str()) * parameter_reader->D2R; //geodetic longitude deg to radius
        gps_unit[3] = std::atof(csv_gps_info[i][4].c_str()); // geodetic height meter
        gps_unit[4] = std::atof(csv_gps_info[i][3].c_str()); // latitude standard deviation meter
        gps_unit[5] = std::atof(csv_gps_info[i][3].c_str()); // longitude standard deviation meter
        gps_unit[6] =std::atof(csv_gps_info[i][5].c_str());  // height standard deviation meter
        gps_unit[7] = std::atof(csv_gps_info[i][6].c_str()); // height standard deviation meter
        gps_unit[8] = std::atof(csv_gps_info[i][7].c_str()); // course(dgr)
        gps_unit[9] = std::atof(csv_gps_info[i][8].c_str()); // speed(m/s)
        //std::cout << "gps_unit=" << gps_unit <<std::endl;
        gps_units_vec.push_back(gps_unit);
    }

    // input imu format
    for (int i = 0, n = csv_imu_raw.size(); i < n; i++) {
        Eigen::Matrix<double, 7, 1> imu_raw_units;
        imu_raw_units[0] = std::atof(csv_imu_raw[i][0].c_str()); //imu second
        if ((gps_units_vec[0][0] > imu_raw_units[0]))
            continue;
        imu_raw_units[1] = std::atof(csv_imu_raw[i][1].c_str()); // acc
        imu_raw_units[2] = std::atof(csv_imu_raw[i][2].c_str()); // acc
        imu_raw_units[3] = std::atof(csv_imu_raw[i][3].c_str()); // acc
        imu_raw_units[4] = std::atof(csv_imu_raw[i][4].c_str()); // gyro
        imu_raw_units[5] = std::atof(csv_imu_raw[i][5].c_str()); // gyro
        imu_raw_units[6] = std::atof(csv_imu_raw[i][6].c_str()); // gyro
        imu_raw_units_vec.push_back(imu_raw_units);
    }

#if 0
    for (int i = 0, n = csv_groundtruth.size(); i < n; i++) {
        Eigen::Matrix<double, 10, 1> gt_units;
        gt_units[0] =
            std::atof(csv_groundtruth[i][1].c_str()); // GNSS seconds of week
        gt_units[1] =
            std::atof(csv_groundtruth[i][2].c_str()); // geodetic latitude deg
        gt_units[2] =
            std::atof(csv_groundtruth[i][3].c_str()); // geodetic longitude deg
        gt_units[3] =
            std::atof(csv_groundtruth[i][4].c_str()); // geodetic height meter
        gt_units[4] = std::atof(
            csv_groundtruth[i][5].c_str()); // velocity in north direction m/s
        gt_units[5] = std::atof(
            csv_groundtruth[i][6].c_str()); // velocity in east direction m/s
        gt_units[6] = std::atof(
            csv_groundtruth[i][7].c_str());                     // velocity in down direction m/s
        gt_units[7] = std::atof(csv_groundtruth[i][8].c_str()); // roll attitude
                                                                // deg
        gt_units[8] =
            std::atof(csv_groundtruth[i][9].c_str());            // pitch attitude deg
        gt_units[9] = std::atof(csv_groundtruth[i][10].c_str()); // yaw attitude
                                                                 // deg
        ground_truth_vec.emplace_back(gt_units);
    }
#endif

    // Used to convert first world axis align gravity
    //[WORLD]維持座標垂直於地面
    double Gw_gnss = 0;
    std::vector<Eigen::Matrix<double, 7, 1>>::iterator imu_vec_iterator =  imu_raw_units_vec.begin();

    int count_ignore = 0;
    double last_gps_timestamp = -1;
    for (int i = 0; i < gps_units_vec.size(); i++) {
        count_ignore++;
        double gps_time = gps_units_vec[i][0];

        // std::cout << "======GPS========" << std::endl;
        std::vector<Eigen::Vector3d> acc_vec;
        std::vector<Eigen::Vector3d> gyro_vec;
        std::vector<double> imu_timestamp_vec;

        while ((*(imu_vec_iterator))[0] <= gps_time && imu_vec_iterator != imu_raw_units_vec.end()) {
            // Inset imu data <= gps_frame
            Eigen::Matrix<double, 7, 1> cur_imu = (*(imu_vec_iterator)); // Time, gyro_xyz, acc_xyz
            // std::cout << std::setprecision(20) << "imu time =" << cur_imu[0] << std::endl;
            acc_vec.push_back(cur_imu.block<3, 1>(1, 0));
            gyro_vec.push_back(cur_imu.tail<3>());
            imu_timestamp_vec.push_back(cur_imu[0]);
            // std::cout << "cur_imu.tail<3>()=" << cur_imu.tail<3>() << std::endl;
            // std::cout << "cur_imu.block<3,1>(1,0)=" << cur_imu.block<3, 1>(1, 0)
            // << std::endl;
            // std::cout << "cur_imu=" << cur_imu << std::endl;
            imu_vec_iterator++;
        }
        last_gps_timestamp = gps_time;

        if (imu_timestamp_vec.size() != 0) {
          FGO_GpsIMU_SOLVER::GNSSFramePtr cur_frame =
            std::make_shared<FGO_GpsIMU_SOLVER::GNSSFrame>();
          cur_frame->v_imu_timestamp = imu_timestamp_vec;
          cur_frame->v_acc = acc_vec;
          cur_frame->v_gyr = gyro_vec;
          cur_frame->timestamp = gps_time;
          cur_frame->ori_blh =gps_units_vec[i].block<3, 1>(1, 0); // radius / radius / meter
          cur_frame->gnss_llh_variance = gps_units_vec[i].block<3, 1>(4, 0); // latitude standard deviation meter / longitude
          std::cout << "gps_units_vec[i] =" << gps_units_vec[i] << std::endl;
          std::cout << "cur_frame->ori_blh =" << cur_frame->ori_blh << std::endl;
          std::cout << "cur_frame->gnss_llh_variance ="
                    << cur_frame->gnss_llh_variance << std::endl;
          
          if (count_ignore > ignore_gps_number &&
              count_ignore % ignore_gps_number != 0 &&
              (gps_units_vec[i][0] - gps_units_vec[0][0] >
               time_to_ignore_gps_number)) {
            cur_frame->ori_blh.setZero();
          }

            FGO_GpsIMU_SOLVER::GNSSResult fgo_sol = fgo_vsolver->ProcessGNSSIMU(cur_frame);
            if (fgo_sol.state == FGO_GpsIMU_SOLVER::TIGHTLY_GINS ||
                fgo_sol.state == FGO_GpsIMU_SOLVER::WAIT_FILL_WINDOW) {
              // std::cout << "fgo_sol pwb=" << fgo_sol.cur_frame->p_wb <<
              // std::endl; std::cout << "fgo_sol station2cur_blh_local_data="
              // << fgo_sol.cur_frame->station2cur_blh_local_data << std::endl;
              // std::cout << "fgo_sol state=" << fgo_sol.state << std::endl;
#if ROS_OUTPUT

                // Pub Path
                double fuse_frame_timestamp = ros::Time::now().toSec();
                // std::cout << "fuse_frame_timestamp=" << fuse_frame_timestamp << std::endl;
                fuse_path.header.seq = 0;
                fuse_path.header.stamp.fromSec(fuse_frame_timestamp);
                fuse_path.header.frame_id = "world";
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp.fromSec(fuse_frame_timestamp);
                pose_stamped.header.frame_id = "fuse";
                pose_stamped.pose.position.x = fgo_sol.cur_frame_result->p_wb[0];
                pose_stamped.pose.position.y = fgo_sol.cur_frame_result->p_wb[1];
                pose_stamped.pose.position.z = fgo_sol.cur_frame_result->p_wb[2];
                pose_stamped.pose.orientation.w = fgo_sol.cur_frame_result->q_wb.unit_quaternion().w();
                pose_stamped.pose.orientation.x = fgo_sol.cur_frame_result->q_wb.unit_quaternion().x();
                pose_stamped.pose.orientation.y = fgo_sol.cur_frame_result->q_wb.unit_quaternion().y();
                pose_stamped.pose.orientation.z = fgo_sol.cur_frame_result->q_wb.unit_quaternion().z();
                fuse_path.poses.emplace_back(pose_stamped);
                pub_fuse_path.publish(fuse_path);

                if (fgo_sol.cur_frame_result->station2cur_blh_local_data[0] != 0 || fgo_sol.cur_frame_result->station2cur_blh_local_data[1] != 0) {
                    gnss_only_path.header.seq = 0;
                    gnss_only_path.header.stamp.fromSec(fuse_frame_timestamp);
                    gnss_only_path.header.frame_id = "world";
                    geometry_msgs::PoseStamped pose_stamped_gps;
                    pose_stamped_gps.header.stamp.fromSec(fuse_frame_timestamp);
                    pose_stamped_gps.header.frame_id = "gps_only";
                    pose_stamped_gps.pose.position.x = fgo_sol.cur_frame_result->station2cur_blh_local_data[0];
                    pose_stamped_gps.pose.position.y = fgo_sol.cur_frame_result->station2cur_blh_local_data[1];
                    pose_stamped_gps.pose.position.z = fgo_sol.cur_frame_result->station2cur_blh_local_data[2];
                    gnss_only_path.poses.emplace_back(pose_stamped_gps);
                    pub_gnss_only_path.publish(gnss_only_path);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
#endif
            }
        } // collect imu data for sync imu and gps data.
    }     // gps_for_loop
    return 0;
}
