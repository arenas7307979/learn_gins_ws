#ifndef VSOLVER_TD_H
#define VSOLVER_TD_H

#include "slam_vio/Vsolver/vertex.h"

namespace Vsolver {

/**
 * 以逆深度形式存储的顶点
 */
class VertexTd : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexTd() : Vertex(1) {}

    virtual std::string TypeInfo() const { return "VertexTd"; }
};

}

#endif
