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

#ifndef IMUFILELOADER_H
#define IMUFILELOADER_H

#include "ob_gins/fileio/fileloader.h"
#include "ob_gins/common/types.h"
#include "iostream"
class ImuFileLoader : public FileLoader {

public:
    ImuFileLoader() = delete;
    ImuFileLoader(const string &filename, int columns, int rate = 200) {
        open(filename, columns, FileLoader::TEXT);

        dt_ = 1.0 / (double) rate;

        imu_.time = 0;
    }

    const IMU &next() {
        imu_pre_ = imu_;

        data_ = load();

        imu_.time = data_[0];
        memcpy(imu_.dtheta.data(), &data_[1], 3 * sizeof(double));
        memcpy(imu_.dvel.data(), &data_[4], 3 * sizeof(double));

        double dt = imu_.time - imu_pre_.time;
        if (dt < 0.1) {
            imu_.dt = dt;
        } else {
            imu_.dt = dt_;
        }

        // 增量形式
        imu_.odovel = data_[7] * imu_.dt;
        // num_imu++;
        // std::cout <<"num_imu=" << num_imu <<std::endl;
        return imu_;
    }

private:
    double dt_;
    int num_imu = 0;
    IMU imu_, imu_pre_;
    vector<double> data_;
};

#endif // IMUFILELOADER_H
