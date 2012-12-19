#ifndef _VERTEX_SE3_OFFSET_PARAMETERS_H_
#define _VERTEX_SE3_OFFSET_PARAMETERS_H_

#include "g2o/core/optimizable_graph.h"

#include "g2o/math_groups/se3quat.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

  class VertexSE3;

  class SE3OffsetParameters: public OptimizableGraph::Parameters {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SE3OffsetParameters();
    void setOffset(const SE3Quat& offset_ = SE3Quat());

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    SE3Quat offset;
    Matrix3d offsetR, inverseOffsetR;
    Vector3d offsetT, inverseOffsetT;
  };

  class SE3OffsetCache: public OptimizableGraph::VertexCache {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SE3OffsetCache(VertexSE3* v, int cacheId, SE3OffsetParameters* params = 0);
    virtual void update();
    SE3OffsetParameters* params;
    SE3Quat se3_w2n, se3_n2w; 

    Matrix<double, 3,4> w2n; // world to sensor transform
    Matrix<double, 3,4> w2l; // world to local
    Matrix<double, 3,4> n2w; // sensor to world
  };

#ifdef G2O_HAVE_OPENGL
  class SE3OffsetCacheDrawAction: public DrawAction{
  public:
    SE3OffsetCacheDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
						HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _cubeSide;
  };
#endif


}

#endif
