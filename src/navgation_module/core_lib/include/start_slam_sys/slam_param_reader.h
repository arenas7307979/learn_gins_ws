#pragma once

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// slam_lib
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <slam_vio/sophus_extra.h>


class ParamReader
{
public:
  ParamReader(const std::string& config_file);
  ~ParamReader();
  const double D2R = (M_PI / 180.0);
  const double R2D = (180.0 / M_PI);

  //#### ParamReader For GNSS-IMU FGO
  double gnss_window_size = 30;
  double gyr_arw = 0;
  double gyr_bias_std = 0;
  double acc_vrw = 0;
  double acc_bias_std = 0;
};
using ParamReaderPtr = std::shared_ptr<ParamReader>;
using ParamReaderConstPtr = std::shared_ptr<const ParamReader>;
