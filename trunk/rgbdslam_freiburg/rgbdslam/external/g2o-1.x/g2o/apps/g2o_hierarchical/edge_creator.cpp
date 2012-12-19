#include "g2o/core/factory.h"
#include "edge_creator.h"

namespace g2o {

  using namespace std;

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType, const std::vector<int>& cacheIds) {

    EntryMap::iterator it = _vertexToEdgeMap.find(vertexTypes);
    if (it!=_vertexToEdgeMap.end())
      it->second = edgeType;
    else
      _vertexToEdgeMap.insert(make_pair(vertexTypes,EdgeCreatorEntry(edgeType, cacheIds)));
    return true;
  }

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType) {
    return addAssociation(vertexTypes, edgeType, std::vector<int>());
  }

  bool EdgeCreator::removeAssociation(std::string vertexTypes){
    EntryMap::iterator it = _vertexToEdgeMap.find(vertexTypes);
    if (it==_vertexToEdgeMap.end())
      return false;
    _vertexToEdgeMap.erase(it);
    return true;
  }


  OptimizableGraph::Edge* EdgeCreator::createEdge(std::vector<OptimizableGraph::Vertex*>& vertices ){
    std::stringstream key;
    Factory* factory=Factory::instance();
    for (size_t i=0; i<vertices.size(); i++){
      key << factory->tag(vertices[i]) << ";";
    }
    EntryMap::iterator it=_vertexToEdgeMap.find(key.str());
    if (it==_vertexToEdgeMap.end())
      return 0;
    HyperGraph::HyperGraphElement* element=factory->construct(it->second._edgeTypeName);
    if (! element)
      return 0;
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(element);
    for (size_t i=0; i<it->second._edgeCacheIds.size(); i++){
      e->setCacheId(i,it->second._edgeCacheIds[i]);
    }
    assert (e);
    for (size_t i=0; i<vertices.size(); i++)
      e->vertices()[i]=vertices[i];
    return e;
  }

}

