#include "edge_se3_trackxyz.h"
#include "se3_offset_parameters.h"
#include <iostream>

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSE3TrackXYZ::EdgeSE3TrackXYZ() : BaseBinaryEdge<3, Vector3d, VertexSE3, VertexTrackXYZ>() {
    information().setIdentity();
    J.fill(0);
    J.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    _cacheIds.resize(2,-1);
  }


  OptimizableGraph::VertexCache* EdgeSE3TrackXYZ::createCache(int vertexNum, OptimizableGraph* optimizer) {
    int cId=getCacheId(vertexNum);
    if (cId>=0){
      SE3OffsetParameters* p = dynamic_cast <SE3OffsetParameters*> (optimizer->parameters(cId));
      if (!p){
	cerr << "invalid cache ids: " << cId << endl;
	return 0;
      }
      SE3OffsetCache* vcache= new SE3OffsetCache((VertexSE3*)_vertices[0], cId, p);
      return vcache;
    }
    return 0;
  }


  bool EdgeSE3TrackXYZ::read(std::istream& is) {
    is >> _cacheIds[0];
    // measured keypoint
    Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
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
    } 
    return true;
  }

  bool EdgeSE3TrackXYZ::write(std::ostream& os) const {
    os << _cacheIds[0] << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3TrackXYZ::computeError() {
    // from cam to point (track)
    VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);
    SE3OffsetCache* vcache = (SE3OffsetCache*)cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Vector4d px;
    px.head<3>()=point->estimate();
    px(3)=1.;
    Eigen::Vector3d perr = vcache->w2n * px;

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
    //    std::cout << _error << std::endl << std::endl;
  }

  void EdgeSE3TrackXYZ::linearizeOplus() {
    VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexTrackXYZ *vp = static_cast<VertexTrackXYZ *>(_vertices[1]);

    SE3OffsetCache* vcache = (SE3OffsetCache*)cam->getCache(_cacheIds[0]);
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

    Eigen::Matrix<double,3,9> Jhom = vcache->params->inverseOffsetR  * J;

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }


  bool EdgeSE3TrackXYZ::setMeasurementFromState(){
    VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = static_cast<VertexTrackXYZ*>(_vertices[1]);

    // calculate the projection
    const Vector3d &pt = point->estimate();
    SE3OffsetCache* vcache = (SE3OffsetCache*) cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }

    Eigen::Vector4d ppt(pt(0),pt(1),pt(2),1.0);
    Eigen::Vector3d perr = vcache->w2n*ppt;
    _measurement = perr;
    return true;
  }


  void EdgeSE3TrackXYZ::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");

    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexTrackXYZ *point = dynamic_cast<VertexTrackXYZ*>(_vertices[1]);
    SE3OffsetCache* vcache = (SE3OffsetCache* ) cam->getCache(_cacheIds[0]);
    if (! vcache){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3OffsetParameters* params=vcache->params;
    Eigen::Vector3d p=_measurement;
    point->setEstimate(cam->estimate() * (params->offsetR *p + params->offsetT));
  }

}
