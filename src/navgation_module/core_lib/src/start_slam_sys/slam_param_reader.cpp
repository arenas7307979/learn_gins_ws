
#include "start_slam_sys/slam_param_reader.h"

ParamReader::ParamReader(const std::string &config_file) {
  YAML::Node slam_param_data = YAML::LoadFile(config_file);

  //GNSS-IMU FGO parameters
  gnss_window_size = slam_param_data["gps_fgo"]["gnss_window_size"].as<double>();
  gyr_arw = slam_param_data["gps_fgo"]["arw"].as<double>() * D2R / 60.0;
  gyr_bias_std = slam_param_data["gps_fgo"]["gbstd"].as<double>() * D2R / 3600.0;
  acc_vrw = slam_param_data["gps_fgo"]["vrw"].as<double>() / 60.0;
  acc_bias_std = slam_param_data["gps_fgo"]["abstd"].as<double>() * 1.0e-5;
}

ParamReader::~ParamReader(){};
