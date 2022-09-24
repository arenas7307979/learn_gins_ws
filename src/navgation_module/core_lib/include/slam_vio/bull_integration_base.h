/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <sophus/so3.hpp>

// #include <ceres/ceres.h>
enum BullStateOrder { B_O_P = 0, B_O_R = 6, B_O_V = 3, B_O_BA = 12, B_O_BG = 9 };
enum BullNoiseOrder { B_O_AN = 3, B_O_GN = 0, B_O_AW = 9, B_O_GW = 6 }; 
class BullIntegrationBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BullIntegrationBase() = delete;
    BullIntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg,
                    double acc_n, double gyr_n, double acc_w, double gyr_w);
    // copy constructor
    BullIntegrationBase(const std::shared_ptr<BullIntegrationBase> ori_integrartion);
    ~BullIntegrationBase(){};
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);

    void NormalEulerIntegration(double _dt, const Eigen::Vector3d &_acc_0,
                             const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1,
                             const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p,
                             const Sophus::SO3d &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba,
                             const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
                             Sophus::SO3d &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba,
                             Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1);

    // for time-offset(cam and imu) calibration
    Eigen::Matrix<double, 15, 1>
    evaluate(const Eigen::Vector3d &Pi, const Sophus::SO3d &Qi, const Eigen::Vector3d &Vi,
             const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
             const Sophus::SO3d &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj,
             const Eigen::Vector3d &Bgj, const double &td_para, const Eigen::Vector3d &Gw,
             const Eigen::Vector3d &w_begin, const Eigen::Vector3d &w_end,
             const Eigen::Vector3d &acc_begin, const Eigen::Vector3d &acc_end,
             const double &t_begin, const double &t_end, const double &td_fix_last);

    // for original evaluate
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Sophus::SO3d &Qi,
                                          const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
                                          const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                          const Sophus::SO3d &Qj, const Eigen::Vector3d &Vj,
                                          const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj,
                                          const Eigen::Vector3d &Gw);

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 12, 12> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Sophus::SO3d delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

    Eigen::Vector3d last_un_gyr_dt;
    Eigen::Vector3d midPointIntegration_last_un_gyr_dt;
};
using BullIntegrationBasePtr = std::shared_ptr<BullIntegrationBase>;
using BullIntegrationBaseConstPtr = std::shared_ptr<const BullIntegrationBase>;
