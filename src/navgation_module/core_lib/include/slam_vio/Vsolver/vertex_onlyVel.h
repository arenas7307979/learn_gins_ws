#ifndef VSOLVER_VELVERTEX_H
#define VSOLVER_VELVERTEX_H
#include "slam_vio/Vsolver/vertex.h"
namespace Vsolver
{
/**
 * @brief 以xyz形式参数化的顶点
 */
class VertexVel : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexVel() : Vertex(3) {}

    std::string TypeInfo() const { return "VertexVel"; }
};

} // namespace Vsolver

#endif
