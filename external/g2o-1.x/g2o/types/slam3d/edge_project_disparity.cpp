#include "edge_project_disparity.h"

#include "camera_parameters.h"
#include <iostream>
#include <iomanip>

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeProjectDisparity::EdgeProjectDisparity() : BaseBinaryEdge<3, Vector3d, VertexSE3, VertexTrackXYZ>() {
    information().setIdentity();
    information()(2,2)=1000.;
    J.fill(0);
    J.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    _cacheIds.resize(2,-1);
  }

  OptimizableGraph::VertexCache* EdgeProjectDisparity::createCache(int vertexNum, OptimizableGraph* optimizer) {
    int cId=getCacheId(vertexNum);
    if (cId>=0){
      CameraParameters* p = dynamic_cast <CameraParameters*> (optimizer->parameters(cId));
      if (!p){
	cerr << "invalid cache ids: " << cId << endl;
	return 0;
      }
      VertexCameraCache* vcache= new VertexCameraCache((VertexSE3*)_vertices[0], cId, p);
      return vcache;
    }
    return 0;
  }

  bool EdgeProjectDisparity::read(std::istream& is) {
    is >> _cacheIds[0];
    // measured keypoint
    Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
    if (is.bad())
      return false;
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
	is >> information()(i,j);
	if (i!=j)
	  information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
      information()(2,2)=1000.;
    }
    return true;
  }

  bool EdgeProjectDisparity::write(std::ostream& os) const {
    os << _cacheIds[0] << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeProjectDisparity::computeError() {
    VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);
    const Vector3d& pt = point->estimate();
    //Eigen::Vector4d ppt(pt(0),pt(1),pt(2),1.0);
    
    VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }
    
    Vector4d px;
    px.head<3>()=pt;
    px(3)=1.;
    Eigen::Vector3d p = vcache->w2i * px;

    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = 1/p(2);
    
    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
  }

#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN

  void EdgeProjectDisparity::linearizeOplus() {
    VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexTrackXYZ *vp = static_cast<VertexTrackXYZ *>(_vertices[1]);

    VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    const Eigen::Vector3d& pt = vp->estimate();
    Eigen::Vector4d ptx;
    ptx.head<3>()=pt;
    ptx[3]=1.;


    Eigen::Vector3d Zcam = vcache->w2l*ptx;

    //  J(0,3) = -0.0;
    J(0,4) = -2*Zcam(2);
    J(0,5) = 2*Zcam(1);

    J(1,3) = 2*Zcam(2);
    //  J(1,4) = -0.0;
    J(1,5) = -2*Zcam(0);

    J(2,3) = -2*Zcam(1);
    J(2,4) = 2*Zcam(0);
    //  J(2,5) = -0.0;

    J.block<3,3>(0,6) = vcache->w2l.block<3,3>(0,0);

    Eigen::Matrix<double,3,9> Jprime = vcache->params->Kcam_inverseOffsetR  * J;
    Eigen::Matrix<double, 3, 9> Jhom;
    Eigen::Vector3d Zprime = vcache->w2i * ptx;

    Jhom.block<2,9>(0,0) = 1/(Zprime(2)*Zprime(2)) * (Jprime.block<2,9>(0,0)*Zprime(2) - Zprime.head<2>() * Jprime.block<1,9>(2,0));
    Jhom.block<1,9>(2,0) = - 1/(Zprime(2)*Zprime(2)) * Jprime.block<1,9>(2,0);

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }

#endif

  bool EdgeProjectDisparity::setMeasurementFromState(){
    VertexSE3 *cam = static_cast< VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);
    const Vector3d &pt = point->estimate();

    VertexCameraCache* vcache = (VertexCameraCache*) cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Eigen::Vector4d ppt(pt(0),pt(1),pt(2),1.0);
    Eigen::Vector3d p = vcache->w2i*ppt;

    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = 1/p(2);

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _measurement = perr;
    return true;
  }

  void EdgeProjectDisparity::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");
    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = dynamic_cast<VertexTrackXYZ*>(_vertices[1]);

    VertexCameraCache* vcache = (VertexCameraCache* ) cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }
    CameraParameters* params=vcache->params;
    const Eigen::Matrix<double, 3, 3>& invKcam = params->invKcam;
    Eigen::Vector3d p;
    double w=1./_measurement(2);
    p.head<2>() = _measurement.head<2>()*w;
    p(2) = w;
    p = invKcam * p;
    p = cam->estimate() * (params->offsetR *p + params->offsetT);
    point->setEstimate(p);
  }


#ifdef G2O_HAVE_OPENGL
  EdgeProjectDisparityDrawAction::EdgeProjectDisparityDrawAction(): DrawAction(typeid(EdgeProjectDisparity).name()){}

  HyperGraphElementAction* EdgeProjectDisparityDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
								HyperGraphElementAction::Parameters* /* params_ */){
    return 0;
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeProjectDisparity* e =  static_cast<EdgeProjectDisparity*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
    VertexTrackXYZ* toEdge   = static_cast<VertexTrackXYZ*>(e->vertices()[1]);
    glColor3f(0.4,0.4,0.2);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),fromEdge->estimate().translation().z());
    glVertex3f(toEdge->estimate().x(),toEdge->estimate().y(),toEdge->estimate().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

}
