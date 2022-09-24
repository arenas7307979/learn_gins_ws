#ifndef VSOLVER_BIASVERTEX_H
#define VSOLVER_BIASVERTEX_H

#include <memory>
#include "slam_vio/Vsolver/vertex.h"

namespace Vsolver {
/**
 * SpeedBias vertex
 * parameters: ba, bg 6 DoF
 * 
 */
class VertexBias : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexBias() : Vertex(6) {}

    std::string TypeInfo() const {
        return "VertexBias";
    }

};

}
#endif
