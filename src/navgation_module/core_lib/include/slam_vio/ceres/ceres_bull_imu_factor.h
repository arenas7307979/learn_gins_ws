/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "slam_vio/bull_integration_base.h"
#include <ceres/ceres.h>

class CERES_BULL_IMUFactor : public ceres::SizedCostFunction<15, 7, 3, 6, 7, 3, 6>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CERES_BULL_IMUFactor(BullIntegrationBasePtr _pre_integration, const Eigen::Vector3d& Gw_);
    ~CERES_BULL_IMUFactor(){};
    virtual bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const;
    // void check(double **parameters);
    BullIntegrationBasePtr pre_integration;
    Eigen::Vector3d Gw;
};

class CERES_LAST_IMU_BIAS_ERROR : public ceres::SizedCostFunction<6, 6>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CERES_LAST_IMU_BIAS_ERROR(){};
  ~CERES_LAST_IMU_BIAS_ERROR(){};
  virtual bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const;

private:
  Eigen::Vector3d ba, bg;
  static constexpr double IMU_GRY_BIAS_STD = 7200 / 3600.0 * M_PI / 180.0; // 7200 deg / hr
  static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;               // 20000 mGal
};
