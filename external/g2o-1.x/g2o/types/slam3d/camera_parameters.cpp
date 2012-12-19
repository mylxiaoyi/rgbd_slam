#include "camera_parameters.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

// DEPTH CAM

  CameraParameters::CameraParameters(){
    setId(-1);
    setKcam(1,1,0.5,0.5);
    setOffset();
  }

  void CameraParameters::setOffset(const SE3Quat& offset_){
    offset = offset_;
    offsetR = offset.rotation().toRotationMatrix();
    offsetT = offset.translation();
    inverseOffsetR=offsetR.transpose();
    inverseOffsetT=-inverseOffsetR*offsetT;
    Kcam_inverseOffsetR = Kcam * inverseOffsetR;
  }

  void CameraParameters::setKcam(double fx, double fy, double cx, double cy){
    Kcam.setZero();
    Kcam(0,0) = fx;
    Kcam(1,1) = fy;
    Kcam(0,2) = cx;
    Kcam(1,2) = cy;
    Kcam(2,2) = 1.0;
    invKcam = Kcam.inverse();
    Kcam_inverseOffsetR = Kcam * inverseOffsetR;
  }


  bool CameraParameters::read(std::istream& is) {
    Vector7d off;
    for (int i=0; i<7; i++)
      is >> off[i];
    setOffset(SE3Quat(off.head<6>())); // ignore w and re-compute it by normalizing, otherwise an assertion in se3quat.h is triggered
    double fx,fy,cx,cy;
    is >> fx >> fy >> cx >> cy;
    setKcam(fx,fy,cx,cy);
    return is.good();
  }
  
  bool CameraParameters::write(std::ostream& os) const {
    Vector7d off = offset.toVector();
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    os << Kcam(0,0) << " ";
    os << Kcam(1,1) << " ";
    os << Kcam(0,2) << " ";
    os << Kcam(1,2) << " ";
    return os.good();
  }



  VertexCameraCache::VertexCameraCache(VertexSE3* v, int cacheId, CameraParameters* params_):
    SE3OffsetCache(v,cacheId,params_) {
    if (! params_) {
      std::cerr << "fatal, no parameters assigned" << std::endl;
    }
    params = params_;
  }
  
  void VertexCameraCache::update(){
    SE3OffsetCache::update();
    w2i=params->Kcam * w2n;
  }  

#ifdef G2O_HAVE_OPENGL
  static void drawMyPyramid(float height, float side){
    Vector3f p[6];
    p[0] << 0, 0., 0.;
    p[1] << -side, -side, height;
    p[2] << -side,  side, height;
    p[3] << side,  side, height;
    p[4] << side, -side, height;
    p[5] << -side, -side, height;

    glBegin(GL_TRIANGLES);
    for (int i = 1; i < 5; ++i) {
      Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
      glNormal3f(normal.x(), normal.y(), normal.z());
      glVertex3f(p[0].x(), p[0].y(), p[0].z());
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
      glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
    }
    glEnd();
  }

  CameraCacheDrawAction::CameraCacheDrawAction(): DrawAction(typeid(VertexCameraCache).name()){
    _previousParams = (DrawAction::Parameters*)0x42;
    refreshPropertyPtrs(0);
  }


  bool CameraCacheDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _cameraZ = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_Z", .05);
      _cameraSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CAMERA_SIDE", .05);
      
    } else {
      _cameraZ = 0;
      _cameraSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* CameraCacheDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							   HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return 0;
    VertexCameraCache* that = static_cast<VertexCameraCache*>(element);
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    glPushMatrix();
    Vector3d offsetT=that->params->offset.translation();
    AngleAxisd aa(that->params->offset.rotation());
    glTranslatef(offsetT.x(), offsetT.y(), offsetT.z());
    glRotatef(RAD2DEG(aa.angle()),aa.axis().x(),aa.axis().y(),aa.axis().z());
    if (_cameraZ && _cameraSide)
      drawMyPyramid(_cameraZ->value(), _cameraSide->value());
    glPopMatrix();

    return this;
  }
#endif

}
