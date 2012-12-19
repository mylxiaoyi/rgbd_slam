#include "se3_offset_parameters.h"

#include "vertex_se3_quat.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

// DEPTH CAM

  SE3OffsetParameters::SE3OffsetParameters(){
    setOffset();
  }

  void SE3OffsetParameters::setOffset(const SE3Quat& offset_){
    offset = offset_;
    offsetR = offset.rotation().toRotationMatrix();
    offsetT = offset.translation();
    inverseOffsetR=offsetR.transpose();
    inverseOffsetT=-inverseOffsetR*offsetT;
  }


  bool SE3OffsetParameters::read(std::istream& is) {
    Vector7d off;
    for (int i=0; i<7; i++)
      is >> off[i];
    setOffset(SE3Quat(off));
    return is.good();
  }
  
  bool SE3OffsetParameters::write(std::ostream& os) const {
    Vector7d off = offset.toVector();
    for (int i=0; i<7; i++)
      os << off[i] << " ";
    return os.good();
  }


  SE3OffsetCache::SE3OffsetCache(VertexSE3* v, int cacheId, SE3OffsetParameters* params_):
    OptimizableGraph::VertexCache(v,cacheId) {
    if (! params_) {
      std::cerr << "fatal, no parameters assigned" << std::endl;
    }
    params = params_;
  }
  
  void SE3OffsetCache::update(){
    const VertexSE3* v=(const VertexSE3*) _vertex;
    se3_n2w=v->estimate()*params->offset;

    n2w.block<3,3>(0,0) = se3_n2w.rotation().toRotationMatrix();
    n2w.col(3) = se3_n2w.translation();

    se3_w2n=se3_n2w.inverse();
    w2n.block<3,3>(0,0) = se3_w2n.rotation().toRotationMatrix();
    w2n.col(3) = se3_w2n.translation();

    SE3Quat _w2l=v->estimate().inverse();
    w2l.block<3,3>(0,0) = _w2l.rotation().toRotationMatrix();
    w2l.col(3) = _w2l.translation();
  }  

#ifdef G2O_HAVE_OPENGL

  SE3OffsetCacheDrawAction::SE3OffsetCacheDrawAction(): DrawAction(typeid(SE3OffsetCache).name()){
    _previousParams = (DrawAction::Parameters*)0x42;
    refreshPropertyPtrs(0);
  }


  bool SE3OffsetCacheDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _cubeSide = _previousParams->makeProperty<FloatProperty>(_typeName + "::CUBE_SIDE", .05);
      
    } else {
      _cubeSide = 0;
    }
    return true;
  }

  HyperGraphElementAction* SE3OffsetCacheDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							   HyperGraphElementAction::Parameters* params){
    if (typeid(*element).name()!=_typeName)
      return 0;
    SE3OffsetCache* that = static_cast<SE3OffsetCache*>(element);
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
    // if (_cubeSide)
    //   drawMyPyramid(_cubeSide->value(), _cubeSide->value());
    glPopMatrix();

    return this;
  }
#endif

}
