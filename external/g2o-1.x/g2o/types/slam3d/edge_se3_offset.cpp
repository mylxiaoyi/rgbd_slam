#include "edge_se3_offset.h"
#include "se3_offset_parameters.h"
#include <iostream>

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSE3Offset::EdgeSE3Offset() : BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>() {
    information().setIdentity();
    _cacheIds.resize(2,-1);
  }


  OptimizableGraph::VertexCache* EdgeSE3Offset::createCache(int vertexNum, OptimizableGraph* optimizer) {
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


  bool EdgeSE3Offset::read(std::istream& is) {
    is >> _cacheIds[0] >> _cacheIds[1];
    // measured keypoint
    Vector7d meas;
    for (int i=0; i<7; i++) meas[i];
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

  bool EdgeSE3Offset::write(std::ostream& os) const {
    os << _cacheIds[0] << " " << _cacheIds[1] << " ";
    for (int i=0; i<7; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Offset::computeError() {
    // from cam to point (track)
    VertexSE3 *x1 = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *x2 = static_cast<VertexSE3*>(_vertices[1]);
    //cout << x1->cacheVector().size() << " " <<  x2->cacheVector().size() << endl;
    
    SE3OffsetCache* cache1 = (SE3OffsetCache*)x1->getCache(_cacheIds[0]);
    if (! cache1){
      cerr << "fatal error in retrieving cache " << _cacheIds[0] << endl;
    }
    SE3OffsetCache* cache2 = (SE3OffsetCache*)x2->getCache(_cacheIds[1]);
    if (! cache2){
      cerr << "fatal error in retrieving cache " << _cacheIds[1] << endl;
    }

    SE3Quat delta=_inverseMeasurement*(cache1->se3_w2n)*(cache2->se3_n2w);

    _error.head<3>() = delta.translation();
    // The analytic Jacobians assume the error in this special form (w beeing positive)
    if (delta.rotation().w() < 0.)
      _error.tail<3>() =  - delta.rotation().vec();
    else
      _error.tail<3>() =  delta.rotation().vec();
  }


  bool EdgeSE3Offset::setMeasurementFromState(){
    VertexSE3 *x1 = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *x2 = static_cast<VertexSE3*>(_vertices[1]);
    SE3OffsetCache* cache1 = (SE3OffsetCache*)x1->getCache(_cacheIds[0]);
    if (! cache1){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3OffsetCache* cache2 = (SE3OffsetCache*)x2->getCache(_cacheIds[1]);
    if (! cache2){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3Quat delta=(cache1->se3_w2n)*(cache1->se3_n2w);
    setMeasurement(delta);
    return true;
  }


  void EdgeSE3Offset::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);

    SE3OffsetCache* cacheFrom = (SE3OffsetCache*)from->getCache(_cacheIds[0]);
    if (! cacheFrom){
      cerr << "fatal error in retrieving cache" << endl;
    }
    SE3OffsetCache* cacheTo = (SE3OffsetCache*)to->getCache(_cacheIds[1]);
    if (! cacheTo){
      cerr << "fatal error in retrieving cache" << endl;
    }
   

    SE3Quat virtualMeasurement = cacheFrom->params->offset*measurement()*cacheTo->params->offset.inverse();
    
    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * virtualMeasurement);
    } else
      from->setEstimate(to->estimate() * virtualMeasurement.inverse());
  }

}
