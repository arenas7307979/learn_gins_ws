#include "slam_vio/Vsolver/vertex.h"
#include <iostream>


namespace Vsolver {

unsigned long global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension) {
    parameters_.resize(num_dimension, 1);
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = global_vertex_id++;

//    std::cout << "Vertex construct num_dimension: " << num_dimension
//              << " local_dimension: " << local_dimension << " id_: " << id_ << '\n';
}

Vertex::~Vertex() {}

int Vertex::Dimension() const {
    return parameters_.rows();
}

int Vertex::LocalDimension() const
{
  return local_dimension_;
}

void Vertex::Plus(const VecX& delta)
{
  double sum_value = delta.sum();
  if (!std::isnan(sum_value))
  {
      parameters_ += delta;
  }
}
}
