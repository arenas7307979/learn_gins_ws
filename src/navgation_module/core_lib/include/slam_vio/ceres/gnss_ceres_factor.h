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

#ifndef GNSS_CERES_FACTOR_H
#define GNSS_CERES_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

class GnssCeresFactor : public ceres::SizedCostFunction<3, 7>
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GnssCeresFactor() = delete;
    GnssCeresFactor(const Eigen::Vector3d& t_enu_from_gnss_, const Eigen::Vector3d& lever_, const Eigen::Vector3d& gnss_variance_)
    {
        t_enu_from_gnss = t_enu_from_gnss_;
        lever = lever_;
        gnss_variance = gnss_variance_;
    }

    virtual bool Evaluate(const double *const *parameters_raw, double *residuals_raw, double **jacobians_raw) const;

private:
    Eigen::Vector3d t_enu_from_gnss;
    Eigen::Vector3d lever;
    Eigen::Vector3d gnss_variance;
};

#endif // GNSS_CERES_FACTOR_H
