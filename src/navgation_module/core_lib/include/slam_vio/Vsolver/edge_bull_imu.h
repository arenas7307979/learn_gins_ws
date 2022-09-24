#ifndef VSOLVER_IMUEDGE_H
#define VSOLVER_IMUEDGE_H

#include <memory>
#include <string>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "slam_vio/Vsolver/edge.h"
#include "slam_vio/Vsolver/eigen_types.h"
#include "slam_vio/bull_integration_base.h"

namespace Vsolver {

/**
 * 此边是IMU误差，此边为4元边，与之相连的顶点有：Pi Mi Pj Mj
 */
class EdgeBullImu : public Edge
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit EdgeBullImu(BullIntegrationBasePtr _pre_integration,
                   const Eigen::Vector3d& Gw_)
    : Gw(Gw_)
    , Edge(15, 6,
           std::vector<std::string>{ "VertexPose", "VertexSpeed", "VertexBias",
                                     "VertexPose", "VertexSpeed",
                                     "VertexBias" })
  {

    //傳統為E.transpose * sigma.inv() * E,
    //這邊SetInformation 仿造ceres 拆成 [e* sqrt(sigma.inv)].transpose *
    //[e*sqrt(sigma.inv) pre_integration_->covariance = sigma
    pre_integration_ = (_pre_integration);
    SetInformation(pre_integration_->covariance.inverse());
  }

  /// 返回边的类型信息
  virtual std::string TypeInfo() const override { return "EdgeBullImu"; }

  /// 计算残差
  virtual void ComputeResidual() override;

  /// 计算雅可比
  virtual void ComputeJacobians() override;

  //    static void SetGravity(const Vec3 &g) {
  //        gravity_ = g;
  //    }

private:
  BullIntegrationBasePtr pre_integration_;
  Eigen::Vector3d Gw; // just the gravity term between the imu
};

#if 0
class EdgeBullImuTd : public Edge {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit EdgeBullImuTd(BullIntegrationBasePtr _pre_integration, const Eigen::Vector3d &Gw_, const double &last_estimate_td_)
        : Gw(Gw_), last_estimate_td(last_estimate_td_),
          Edge(15, 7, std::vector<std::string>{"VertexPose", "VertexSpeed", "VertexBias", "VertexPose", "VertexSpeed", "VertexBias", "VertexTd"}) {
        //傳統為E.transpose * sigma.inv() * E,
        //這邊SetInformation 仿造ceres 拆成 [e* sqrt(sigma.inv)].transpose *
        //[e*sqrt(sigma.inv) pre_integration_->covariance = sigma
        pre_integration_ = (_pre_integration);
        SetInformation(pre_integration_->covariance.inverse());

        imu_t_begin = pre_integration_->dt_buf[0];
        imu_t_end = pre_integration_->dt_buf.back();
        acc_begin = pre_integration_->acc_buf[0];
        acc_end = pre_integration_->acc_buf.back();
        gyro_begin = pre_integration_->gyr_buf[0];
        gyro_end = pre_integration_->gyr_buf.back();
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "EdgeBullImuTd"; }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    //    static void SetGravity(const Vec3 &g) {
    //        gravity_ = g;
    //    }

  private:
    BullIntegrationBasePtr pre_integration_;
    double last_estimate_td;
    double imu_t_end, imu_t_begin;
    Eigen::Vector3d Gw, acc_end, acc_begin, gyro_begin, gyro_end; // just the gravity term between the imu
};
#endif

} // namespace Vsolver
#endif
