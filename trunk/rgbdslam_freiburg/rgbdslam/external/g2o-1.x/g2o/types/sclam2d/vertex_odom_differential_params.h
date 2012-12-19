#ifndef VERTEX_ODOM_DIFFERENTIAL_PARAMS_H
#define VERTEX_ODOM_DIFFERENTIAL_PARAMS_H

#include "g2o/core/base_vertex.h"

namespace g2o {

  class VertexOdomDifferentialParams: public BaseVertex <3, Vector3d> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexOdomDifferentialParams();
      virtual void setToOrigin() {
        _estimate << 1. , 1., 1.;
      }

      virtual void oplus(double* v) {
        for (int i=0; i<3; i++)
          _estimate(i) += v[i];
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

}

#endif
