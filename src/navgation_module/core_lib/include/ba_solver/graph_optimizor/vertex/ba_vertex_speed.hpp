#pragma once
#include "ba_solver/graph_optimizor/vertex.hpp"

// 定义命名空间为 GraphOptimizor
namespace GraphOptimizor
{

template <typename Scalar>
class BAVertexSpeed final : public VertexBase<Scalar>
{
public:
    /* 构造函数与析构函数 */
    BAVertexSpeed() : VertexBase<Scalar>(3, 3) {}
    ~BAVertexSpeed() {}
};
} // namespace GraphOptimizor