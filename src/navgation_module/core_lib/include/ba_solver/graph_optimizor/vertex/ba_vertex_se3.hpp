#pragma once
#include <sophus/se3.hpp>
#include "ba_solver/graph_optimizor/vertex.hpp"

// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor {

template <typename Scalar>
class BAVertexSE3Pose final : public VertexBase<Scalar>
{
public:
  /* 构造函数与析构函数 */
  BAVertexSE3Pose()
    : VertexBase<Scalar>(7, 6)
  {
  }
  ~BAVertexSE3Pose() {}

public:
  /* 重写参数更新方法，因为四元数更新不是直接相加 */
  virtual void Update(const VectorX<Scalar>& deltaParams) override;
};

/* 类成员方法定义如下 */
/* 重写参数更新方法，因为四元数更新不是直接相加 */
template <typename Scalar>
void BAVertexSE3Pose<Scalar>::Update(const VectorX<Scalar>& deltaParams)
{
  VectorX<Scalar>& param = this->GetParameters();
  param[4] += deltaParams[0];
  param[5] += deltaParams[1];
  param[6] += deltaParams[2];

  Eigen::Map<Sophus::SO3<Scalar>> q_so3(param.data());
  q_so3 = q_so3 *
    Sophus::SO3<Scalar>::exp(Eigen::Matrix<Scalar, 3, 1>(
      deltaParams[3], deltaParams[4],
      deltaParams[5])); // right multiplication with so3'

  param[0] = q_so3.unit_quaternion().x();
  param[1] = q_so3.unit_quaternion().y();
  param[2] = q_so3.unit_quaternion().z();
  param[3] = q_so3.unit_quaternion().w();
}
} // namespace GraphOptimizor