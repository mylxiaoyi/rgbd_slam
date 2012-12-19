#ifndef _CAMERA_PARAMETERS_H_
#define _CAMERA_PARAMETERS_H_

#include "g2o/math_groups/se3quat.h"
#include "g2o/core/hyper_graph_action.h"
#include "se3_offset_parameters.h"

namespace g2o {

  class CameraParameters: public SE3OffsetParameters {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      CameraParameters();
      void setKcam(double fx, double fy, double cx, double cy);
      void setOffset(const SE3Quat& offset_ = SE3Quat());

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      Eigen::Matrix3d Kcam;
      Eigen::Matrix3d invKcam;
      Eigen::Matrix3d Kcam_inverseOffsetR;
  };

  class VertexCameraCache: public SE3OffsetCache {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexCameraCache(VertexSE3* v, int cacheId, CameraParameters* params = 0);
      virtual void update();
      Matrix<double, 3,4> w2i; // world to image transform
      CameraParameters* params;
  };

#ifdef G2O_HAVE_OPENGL
  class CameraCacheDrawAction: public DrawAction{
    public:
      CameraCacheDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ );
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _cameraZ, *_cameraSide;
  };
#endif

} // end namespace

#endif
