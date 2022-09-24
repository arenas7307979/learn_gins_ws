#ifndef VSOLVER_SPEEDBIASVERTEX_H
#define VSOLVER_SPEEDBIASVERTEX_H

#include <memory>
#include "slam_vio/Vsolver/vertex.h"

namespace Vsolver {
/**
 * SpeedBias vertex
 * parameters: v, ba, bg 9 DoF
 * 
 */
class VertexSpeedBias : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexSpeedBias() : Vertex(9) {}

    std::string TypeInfo() const {
        return "VertexSpeedBias";
    }

};

}
#endif
