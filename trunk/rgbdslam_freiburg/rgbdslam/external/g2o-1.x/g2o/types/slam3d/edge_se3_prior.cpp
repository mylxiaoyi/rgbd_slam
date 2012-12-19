#include "edge_se3_prior.h"
#include "se3_offset_parameters.h"
#include <iostream>

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSE3Prior::EdgeSE3Prior() : BaseUnaryEdge<6, SE3Quat, VertexSE3>() {
    information().setIdentity();
    _cacheIds.resize(1,-1);
  }


  OptimizableGraph::VertexCache* EdgeSE3Prior::createCache(int vertexNum, OptimizableGraph* optimizer) {
    if (vertexNum>=(int)vertices().size()){
      cerr << "vertexNum out of edge range vnum: " << vertexNum << " range =" << vertices().size() << endl;
      return 0;
    }
    int cId=getCacheId(vertexNum);
    if (cId>=0){
      SE3OffsetParameters* p = dynamic_cast <SE3OffsetParameters*> (optimizer->parameters(cId));
      if (!p){
	cerr << "invalid cache ids: " << cId << endl;
	return 0;
      }
      SE3OffsetCache* vcache= new SE3OffsetCache((VertexSE3*)_vertices[vertexNum], cId, p);
      return vcache;
    }
    return 0;
  }


  bool EdgeSE3Prior::read(std::istream& is) {
    is >> _cacheIds[0];
    // measured keypoint
    Vector7d meas;
    for (int i=0; i<7; i++) is >> meas[i];
    setMeasurement(SE3Quat(meas));
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
    } 
    return true;
  }

  bool EdgeSE3Prior::write(std::ostream& os) const {
    os << _cacheIds[0] <<  " ";
    for (int i=0; i<7; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Prior::computeError() {
    // from cam to point (track)
    VertexSE3 *x1 = static_cast<VertexSE3*>(_vertices[0]);
    //cout << x1->cacheVector().size() << " " <<  x2->cacheVector().size() << endl;
    
    SE3OffsetCache* cache1 = (SE3OffsetCache*)x1->getCache(_cacheIds[0]);
    if (! cache1){
      cerr << "fatal error in retrieving cache " << _cacheIds[0] << endl;
    }

    SE3Quat delta=_inverseMeasurement*(cache1->se3_n2w);

    _error.head<3>() = delta.translation();
    // The analytic Jacobians assume the error in this special form (w beeing positive)
    if (delta.rotation().w() < 0.)
      _error.tail<3>() =  - delta.rotation().vec();
    else
      _error.tail<3>() =  delta.rotation().vec();
  }


  bool EdgeSE3Prior::setMeasurementFromState(){
    VertexSE3 *x1 = static_cast<VertexSE3*>(_vertices[0]);
    SE3OffsetCache* cache1 = (SE3OffsetCache*)x1->getCache(_cacheIds[0]);
    if (! cache1){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3Quat delta=(cache1->se3_n2w);
    setMeasurement(delta);
    return true;
  }


  void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);

    SE3OffsetCache* cache = (SE3OffsetCache*)v->getCache(_cacheIds[0]);
    if (! cache){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3Quat newEstimate=cache->params->offset.inverse()*measurement();
    if (_information.block<3,3>(0,0).squaredNorm()==0){ // do not set translation
      newEstimate.setTranslation(v->estimate().translation());
    }
    if (_information.block<3,3>(3,3).squaredNorm()==0){ // do not set rotation
      newEstimate.setRotation(v->estimate().rotation());
    }
    v->setEstimate(newEstimate);
  }

}
