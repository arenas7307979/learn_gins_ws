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

#ifndef IMU_ERROR_FACTOR_H
#define IMU_ERROR_FACTOR_H

#include "ob_gins/preintegration/preintegration_base.h"

#include <ceres/ceres.h>

class ImuErrorFactor : public ceres::CostFunction {

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit ImuErrorFactor(std::shared_ptr<PreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {

        *mutable_parameter_block_sizes() = preintegration_->imuErrorNumBlocksParameters();
        set_num_residuals(preintegration_->imuErrorNumResiduals());
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {

        // parameters: vel[3], bg[3], ba[3]

        preintegration_->imuErrorEvaluate(parameters, residuals);

        if (jacobians) {
            if (jacobians[0]) {
                preintegration_->imuErrorJacobian(jacobians[0]);
            }
        }

        return true;
    }

private:
    std::shared_ptr<PreintegrationBase> preintegration_;
};

#endif // IMU_ERROR_FACTOR_H
