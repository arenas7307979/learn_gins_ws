#pragma once

#include "ba_solver/graph_optimizor/vertex.hpp"

// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor
{
//bias acc, bias gyro
template <typename Scalar>
class BAVertexBias final : public VertexBase<Scalar>
{
public:
    /* 构造函数与析构函数 */
    BAVertexBias() : VertexBase<Scalar>(6, 6) {}
    ~BAVertexBias() {}
};
} // namespace GraphOptimizor