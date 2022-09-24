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

#include "slam_vio/ceres/gnss_ceres_factor.h"
#include "slam_vio/sophus_extra.h"

bool GnssCeresFactor::Evaluate(double const *const *parameters_raw,
                               double *residuals_raw,
                               double **jacobians) const
{
    Eigen::Map<const Sophus::SO3d> Qwi(parameters_raw[0]);
    Eigen::Map<const Eigen::Vector3d> Pwi(parameters_raw[0] + 4);

    Eigen::Map<Eigen::Matrix<double, 3, 1>> residuals(residuals_raw);
    residuals = Pwi + (Qwi * lever) - t_enu_from_gnss;

    Eigen::Matrix3d weight = Eigen::Matrix3d::Zero();
    //[TODO]
    weight(0, 0) = 1.0 / gnss_variance[0];
    weight(1, 1) = 1.0 / gnss_variance[1];
    weight(2, 2) = 1.0 / gnss_variance[2];
    residuals = weight * residuals;

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            jacobian_pose.block<3, 3>(0, 3) = -Qwi.matrix() * Sophus::SO3d::hat(lever);
            jacobian_pose = weight * jacobian_pose;
        }
    }
    return true;
}
