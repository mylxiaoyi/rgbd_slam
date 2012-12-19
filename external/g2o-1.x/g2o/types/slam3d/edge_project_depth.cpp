#include "edge_project_depth.h"

#include "camera_parameters.h"

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeProjectDepth::EdgeProjectDepth() : BaseBinaryEdge<3, Vector3d, VertexSE3, VertexTrackXYZ>() {
    information().setIdentity();
    information()(2,2)=100;
    J.fill(0);
    J.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    _cacheIds.resize(2,-1);
  }


  OptimizableGraph::VertexCache* EdgeProjectDepth::createCache(int vertexNum, OptimizableGraph* optimizer) {
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


  bool EdgeProjectDepth::read(std::istream& is) {
    is >> _cacheIds[0];
    // measured keypoint
    Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
    // don't need this if we don't use it in error calculation (???)
    // information matrix is the identity for features, could be changed to allow arbitrary covariances    
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
	is >> information()(i,j);
	if (i!=j)
	  information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
      information()(2,2)=10/_measurement(2); // scale the info by the inverse of the measured depth
    } 
    return true;
  }

  bool EdgeProjectDepth::write(std::ostream& os) const {
    os << _cacheIds[0] << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeProjectDepth::computeError() {
    // from cam to point (track)
    VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);
    VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Vector4d px;
    px.head<3>()=point->estimate();
    px(3)=1.;
    Eigen::Vector3d p = vcache->w2i * px;
    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = p(2);

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
    //    std::cout << _error << std::endl << std::endl;
  }

  void EdgeProjectDepth::linearizeOplus() {
    VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexTrackXYZ *vp = static_cast<VertexTrackXYZ *>(_vertices[1]);

    VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Eigen::Vector3d pt = vp->estimate();
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
    Eigen::Vector3d Zprime = vcache->w2i * ptx;

    Eigen::Matrix<double, 3, 9> Jhom;
    Jhom.block<2,9>(0,0) = 1/(Zprime(2)*Zprime(2)) * (Jprime.block<2,9>(0,0)*Zprime(2) - Zprime.head<2>() * Jprime.block<1,9>(2,0));
    Jhom.block<1,9>(2,0) = Jprime.block<1,9>(2,0);
    /*
    BaseBinaryEdge<3, Vector3d, VertexDepthCam, VertexTrackXYZ>::linearizeOplus();
    std::cout << "Xi numerical" << _jacobianOplusXi << std::endl;
    std::cout << "Xi analytical" << Jhom.block<3,6>(0,0) << std::endl;

    std::cout << "Xj numerical" << _jacobianOplusXj << std::endl;
    std::cout << "Xj analytical" << Jhom.block<3,3>(0,6) << std::endl;
//	printf("prime: %f\n", Zprime(2));
*/
    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }


  bool EdgeProjectDepth::setMeasurementFromState(){
    VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);

    // calculate the projection
    const Vector3d &pt = point->estimate();
    VertexCameraCache* vcache = (VertexCameraCache*) cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Eigen::Vector4d ppt(pt(0),pt(1),pt(2),1.0);
    Eigen::Vector3d p = vcache->w2i*ppt;
    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = p(2);
    _measurement = perr;
    return true;
  }


  void EdgeProjectDepth::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
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
    p(2) = _measurement(2);
    p.head<2>() = _measurement.head<2>()*p(2);
    p=invKcam*p;
    point->setEstimate(cam->estimate() * (params->offsetR *p + params->offsetT));
  }

}
