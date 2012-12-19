// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "optimizable_graph.h"

#include <cassert>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#include "estimate_propagator.h"
#include "factory.h"
#include "solver_property.h"
#include "hyper_graph_action.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/misc.h"

namespace g2o {

  using namespace std;

  OptimizableGraph::VertexCache::VertexCache(Vertex* v, int cacheId): _vertex(v), _cacheId (cacheId) {}

  void OptimizableGraph::VertexCache::update(){}

  OptimizableGraph::Vertex::Vertex() :
    HyperGraph::Vertex(),
    _graph(0), _userData(0), _tempIndex(-1), _fixed(false), _marginalized(false),
    _colInHessian(-1)
  {

  }

  OptimizableGraph::VertexCache* OptimizableGraph::Vertex::getCache(int cacheId) {
    if (cacheId<(int)_cacheVector.size())
      return _cacheVector[cacheId];
    return 0;
  }

  const OptimizableGraph::VertexCache* OptimizableGraph::Vertex::getCache(int cacheId) const
  {
    if (cacheId<(int)_cacheVector.size())
      return _cacheVector[cacheId];
    return 0;
  }
  
  bool OptimizableGraph::Vertex::setCache(VertexCache* cache){
    int cacheId=cache->id();
    if (cache->vertex()!=this)
      cout << "FATAL. mismatch in the vertex cache" << endl;
    if (cacheId>=(int)_cacheVector.size())
      _cacheVector.resize(cacheId+1,0);
    if (_cacheVector[cacheId])
      return false;
    if (cache->vertex()!=this)
      return false;
    _cacheVector[cacheId] = cache;
    //cerr << "update cache: " << id() << "," << cacheId << endl;
    cache->update();
    return true;
  }

  void OptimizableGraph::Vertex::updateCache(){
    for (size_t i=0; i<_cacheVector.size(); i++){
      if (_cacheVector[i])
	_cacheVector[i]->update();
    }
  }

  OptimizableGraph::Vertex::~Vertex()
  {
    for (size_t i=0; i<_cacheVector.size(); i++){
      if (_cacheVector[i])
	delete (_cacheVector[i]);
    }
    delete _userData;
  }
  
  OptimizableGraph::Vertex* OptimizableGraph::Vertex::clone() const
  {
    return 0;
  }

  bool OptimizableGraph::Vertex::setEstimateData(const double *)
  {
    return false;
  }

  bool OptimizableGraph::Vertex::getEstimateData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Vertex::estimateDimension() const
  {
    return -1;
  }

  bool OptimizableGraph::Vertex::setMinimalEstimateData(const double *)
  {
    return false;
  }

  bool OptimizableGraph::Vertex::getMinimalEstimateData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Vertex::minimalEstimateDimension() const
  {
    return -1;
  }


  OptimizableGraph::Edge::Edge() :
    HyperGraph::Edge(),
    _dimension(-1), _level(0), _robustKernel(false), _huberWidth(1.)
  {
  }

  bool OptimizableGraph::Edge::setMeasurementData(const double *)
  {
    return false;
  }

  bool OptimizableGraph::Edge::getMeasurementData(double *) const
  {
    return false;
  }

  int OptimizableGraph::Edge::measurementDimension() const
  {
    return -1;
  }

  bool OptimizableGraph::Edge::setMeasurementFromState(){
    return false;
  }


  OptimizableGraph::Edge* OptimizableGraph::Edge::clone() const
  {
    // TODO
    return 0;
  }

  bool OptimizableGraph::Edge::setCacheId(int vertexNum, int cacheId){
    if ( vertexNum<0 || vertexNum >= (int)vertices().size() )
      return false;
    if (_cacheIds.size()<vertices().size()){
      _cacheIds.resize(vertices().size(), -1);
    }
    _cacheIds[vertexNum]=cacheId;
    return true;
  }

  int OptimizableGraph::Edge::getCacheId(int vertexNum){
    if (vertexNum<0 || vertexNum >= (int) vertices().size())
      return -1;
    if (_cacheIds.size()<vertices().size()){
      _cacheIds.resize(vertices().size(), -1);
    }
    return _cacheIds[vertexNum];
  }


  OptimizableGraph::VertexCache* OptimizableGraph::Edge::createCache(int, OptimizableGraph*) {
    return 0;
  }

  OptimizableGraph::OptimizableGraph()
  {
    _upperGraph=0; _lowerGraph=0; _nextEdgeId = 0; _edge_has_id = false;
    _graphActions.resize(AT_NUM_ELEMENTS);
  }

  OptimizableGraph::~OptimizableGraph()
  {
    clear();
    for (std::map<int, Parameters*>::iterator it=_parametersMap.begin(); it!=_parametersMap.end(); it++){
      if (it->second)
	delete it->second;
    }
  }

  bool OptimizableGraph::addVertex(Vertex* v, Data* userData)
  {
    Vertex* inserted = vertex(v->id());
    if (inserted)
      return false;
    OptimizableGraph::Vertex* ov=dynamic_cast<OptimizableGraph::Vertex*>(v);
    assert(ov && "Vertex does not inherit from OptimizableGraph::Vertex");
    if (ov->_graph != 0) {
      cerr << "FATAL, vertex " << v->id() << " already registered with graph " << ov->_graph << endl;
      return false;
    }
    if (userData)
      ov->setUserData(userData);
    ov->_graph=this;
    HyperGraph::addVertex(v);
    return true;
  }

  bool OptimizableGraph::addEdge(OptimizableGraph::Edge* e)
  {
    OptimizableGraph::Edge* eresult = dynamic_cast<OptimizableGraph::Edge*>(HyperGraph::addEdge(e));
    if (! eresult)
      return false;
    e->_internalId = _nextEdgeId++;
    for (size_t i=0; i<e->vertices().size(); ++i){
      if (e->vertices()[i] == 0) {
        cerr << "FATAL, vertex " << i << " not assigned for edge " << e->id() << endl;
        return false;
      }
      OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)e->vertices()[i];
      int cacheId = e->getCacheId(i);
      if (cacheId<0)
    	continue;
      if (v->getCache(cacheId)) // cache is already there
    	continue;
      VertexCache* cache=e->createCache(i, this);
      if (cache)
    	v->setCache(cache);
      else
    	cerr << "FATAL, cache id provided but no cache created" << endl;
    }
    return true;
  }

  int OptimizableGraph::optimize(int /*iterations*/, bool /*online*/) {return 0;}

double OptimizableGraph::chi2() const
{
  double chi = 0.0;
  for (OptimizableGraph::EdgeSet::const_iterator it = this->edges().begin(); it != this->edges().end(); ++it) {
    const OptimizableGraph::Edge* e = static_cast<const OptimizableGraph::Edge*>(*it);
    chi += e->chi2();
  }
  return chi;
}

void OptimizableGraph::push()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
    v->push();
  }
}

void OptimizableGraph::pop()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v= static_cast<OptimizableGraph::Vertex*>(it->second);
    v->pop();
  }
}

void OptimizableGraph::discardTop()
{
  for (OptimizableGraph::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v= static_cast<OptimizableGraph::Vertex*>(it->second);
    v->discardTop();
  }
}

void OptimizableGraph::push(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->push();
  }
}

void OptimizableGraph::pop(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->pop();
  }
}

void OptimizableGraph::discardTop(HyperGraph::VertexSet& vset)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->discardTop();
  }
}

  void OptimizableGraph::setFixed(HyperGraph::VertexSet& vset, bool fixed)
{
  for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
    v->setFixed(fixed);
  }
}


bool OptimizableGraph::load(istream& is, bool createEdges)
{
  set<string> warnedUnknownTypes;
  stringstream currentLine;
  string token;

  Factory* factory = Factory::instance();
  Vertex* previousVertex = 0;

  while (1) {
    int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1)
      break;
    currentLine >> token;
    if (bytesRead == 0 || token.size() == 0 || token[0] == '#')
      continue;
     
    // do the mapping to an internal type if it matches
    if (_renamedTypesLookup.size() > 0) {
      map<string, string>::const_iterator foundIt = _renamedTypesLookup.find(token);
      if (foundIt != _renamedTypesLookup.end()) {
        token = foundIt->second;
      }
    }

    HyperGraph::HyperGraphElement* element = factory->construct(token);

    if (! element) {
      if (warnedUnknownTypes.count(token) != 1) {
        warnedUnknownTypes.insert(token);
        cerr << CL_RED(__PRETTY_FUNCTION__ << " unknown type: " << token) << endl;
      }
      continue;
    }

    if (dynamic_cast<Vertex*>(element)) { // it's a vertex type
      Vertex* v = static_cast<Vertex*>(element);
      int id;
      currentLine >> id;
      bool r = v->read(currentLine);
      if (! r)
        cerr << __PRETTY_FUNCTION__ << ": Error reading vertex " << token << " " << id << endl;
      v->setId(id);
      if (!addVertex(v)) {
        cerr << __PRETTY_FUNCTION__ << ": Failure adding Vertex, " << token << " " << id << endl;
        delete v;
      } else {
        previousVertex = v;
      }
    }
    else if (dynamic_cast<Edge*>(element)) {
      Edge* e = static_cast<Edge*>(element);
      int numV = e->vertices().size();
      if (_edge_has_id){
	int id;
	currentLine >> id;
	e->setId(id);
      }
      //cerr << PVAR(token) << " " << PVAR(numV) << endl;
      if (numV == 2) { // it's a pairwise / binary edge type which we handle in a special way
        int id1, id2;
        currentLine >> id1 >> id2;
        Vertex* from = vertex(id1);
        Vertex* to = vertex(id2);
        int doInit=0;
        if ((!from || !to) ) {
          if (! createEdges) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to find vertices for edge " << token << " " << id1 << " <-> " << id2 << endl;
            delete e;
          } else {
            if (! from) {
              from=e->createFrom();
              from->setId(id1);
              addVertex(from);
              doInit=2;
            }
            if (! to) {
              to=e->createTo();
              to->setId(id2);
              addVertex(to);
              doInit=1;
            }
          }
        }
        if (from && to) {
          e->read(currentLine);
          e->vertices()[0] = from;
          e->vertices()[1] = to;
          if (!addEdge(e)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge " << token << " " << id1 << " <-> " << id2 << endl;
            delete e;
          } else {
            switch (doInit){
              case 1: 
                {
                  HyperGraph::VertexSet fromSet;
                  fromSet.insert(from);
                  e->initialEstimate(fromSet, to);
                  break;
                }
              case 2:
                {
                  HyperGraph::VertexSet toSet;
                  toSet.insert(to);
                  e->initialEstimate(toSet, from);
                  break;
                }
              default:;
            }
          }
        }
      }
      else {
        vector<int> ids;
        ids.resize(numV);
        for (int l = 0; l < numV; ++l)
          currentLine >> ids[l];
        bool vertsOkay = true;
        for (int l = 0; l < numV; ++l) {
          e->vertices()[l] = vertex(ids[l]);
          if (e->vertices()[l] == 0) {
            vertsOkay = false;
            break;
          }
        }
        if (! vertsOkay) {
          cerr << __PRETTY_FUNCTION__ << ": Unable to find vertices for edge " << token;
          for (int l = 0; l < numV; ++l) {
            if (l > 0)
              cerr << " <->";
            cerr << " " << ids[l];
          }
          delete e;
        } else {
          bool r = e->read(currentLine);
          if (!r || !addEdge(e)) {
            cerr << __PRETTY_FUNCTION__ << ": Unable to add edge " << token; 
            for (int l = 0; l < numV; ++l) {
              if (l > 0)
                cerr << " <->";
              cerr << " " << ids[l];
            }
            delete e;
          }
        }

      }

    }
    else if (dynamic_cast<Data*>(element)) { // reading in the data packet for the vertex
      Data* d = static_cast<Data*>(element);
      //cerr << "read data packet " << token << " vertex " << previousVertex->id() << endl;
      if (! previousVertex) {
        cerr << __PRETTY_FUNCTION__ << ": got data element, but no vertex available" << endl;
        delete d;
      } else {
        bool r = d->read(currentLine);
        if (! r) {
          cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token << " for vertex " << previousVertex->id() << endl;
          delete d;
        } else {
          previousVertex->setUserData(d);
          previousVertex = 0;
        }
      }
    }
    else if (dynamic_cast<Parameters*>(element)) { // reading in the data packet for the vertex
      Parameters* p = static_cast<Parameters*>(element);
      //cerr << "read data packet " << token << " vertex " << previousVertex->id() << endl;
      int pid;
      currentLine >> pid;
      p->setId(pid);
      bool r = p->read(currentLine);
      if (! r) {
	cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token << " for parameter " << pid << endl;
	delete p;
      } else {
	addParameters(p);
	cerr << factory->tag(p) << " " << p->id() << " ";
	p->write(cerr);
	cerr << endl;
      }
    }


  } // while read line
  
  return true;
}

bool OptimizableGraph::load(const char* filename, bool createEdges)
{
  ifstream ifs(filename);
  if (!ifs) {
    cerr << __PRETTY_FUNCTION__ << " unable to open file " << filename << endl;
    return false;
  }
  return load(ifs, createEdges);
}

bool OptimizableGraph::save(const char* filename, int level) const
{
  ofstream ofs(filename);
  if (!ofs)
    return false;
  return save(ofs, level);
}

bool OptimizableGraph::saveParameters(std::ostream& os) const {
  Factory* factory = Factory::instance();
  for (std::map<int, Parameters*>::const_iterator it=_parametersMap.begin(); it!=_parametersMap.end(); ++it){
    if (it->second) {
      os << factory->tag(it->second) << " " << it->second->id() << " ";
      it->second->write(os);
      os << endl;
    }
  }
  return os.good();
}

bool OptimizableGraph::save(ostream& os, int level) const
{
  if (!saveParameters(os))
    return false;
  set<Vertex*, VertexIDCompare> verticesToSave;
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    if (e->level() == level) {
      for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
        verticesToSave.insert(static_cast<OptimizableGraph::Vertex*>(*it));
      }
    }
  }

  for (set<Vertex*, VertexIDCompare>::const_iterator it = verticesToSave.begin(); it != verticesToSave.end(); ++it){
    OptimizableGraph::Vertex* v = *it;
    saveVertex(os, v);
  }

  EdgeContainer edgesToSave;
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    const OptimizableGraph::Edge* e = dynamic_cast<const OptimizableGraph::Edge*>(*it);
    if (e->level() == level)
      edgesToSave.push_back(const_cast<Edge*>(e));
  }
  sort(edgesToSave.begin(), edgesToSave.end(), EdgeIDCompare());

  for (EdgeContainer::const_iterator it = edgesToSave.begin(); it != edgesToSave.end(); ++it) {
    OptimizableGraph::Edge* e = *it;
    saveEdge(os, e);
  }

  return os.good();
}


bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::VertexSet& vset, int level)
{
  if (!saveParameters(os))
    return false;
  for (HyperGraph::VertexSet::const_iterator it=vset.begin(); it!=vset.end(); it++){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
    saveVertex(os, v);
  }
  for (HyperGraph::EdgeSet::const_iterator it = edges().begin(); it != edges().end(); ++it) {
    OptimizableGraph::Edge* e = dynamic_cast< OptimizableGraph::Edge*>(*it);
    if (e->level() != level)
      continue;

    bool verticesInEdge = true;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      if (vset.find(*it) == vset.end()) {
        verticesInEdge = false;
        break;
      }
    }
    if (! verticesInEdge)
      continue;

    saveEdge(os, e);
  }

  return os.good();
}

bool OptimizableGraph::saveSubset(ostream& os, HyperGraph::EdgeSet& eset)
{
  if (!saveParameters(os))
    return false;
  std::set<OptimizableGraph::Vertex*> vset;
  for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
    HyperGraph::Edge* e = *it;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      vset.insert(v);
    }
  }

  for (std::set<OptimizableGraph::Vertex*>::const_iterator it=vset.begin(); it!=vset.end(); ++it){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
    saveVertex(os, v);
  }

  for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
    OptimizableGraph::Edge* e = dynamic_cast< OptimizableGraph::Edge*>(*it);
    saveEdge(os, e);
  }

  return os.good();
}
  
void OptimizableGraph::addGraph(OptimizableGraph* g){
  for (HyperGraph::VertexIDMap::iterator it=g->vertices().begin(); it!=g->vertices().end(); ++it){
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(it->second);
    if (vertex(v->id()))
      continue;
    OptimizableGraph::Vertex* v2=v->clone();
    v2->edges().clear();
    v2->setTempIndex(-1);
    addVertex(v2);
  }
  for (HyperGraph::EdgeSet::iterator it=g->edges().begin(); it!=g->edges().end(); ++it){
    OptimizableGraph::Edge* e = (OptimizableGraph::Edge*)(*it);
    OptimizableGraph::Edge* en = e->clone();
    en->resize(e->vertices().size());
    int cnt = 0;
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) vertex((*it)->id());
      assert(v);
      en->vertices()[cnt++] = v;
    }
    addEdge(en);
  }
}

int OptimizableGraph::maxDimension() const{
  int maxDim=0;
  for (HyperGraph::VertexIDMap::const_iterator it=vertices().begin(); it!=vertices().end(); ++it){
    const OptimizableGraph::Vertex* v= static_cast< const OptimizableGraph::Vertex*>(it->second);
    maxDim = std::max(maxDim, v->dimension());
  }
  return maxDim;
}

void OptimizableGraph::setRenamedTypesFromString(const std::string& types)
{
  Factory* factory = Factory::instance();
  vector<string> typesMap = strSplit(types, ",");
  for (size_t i = 0; i < typesMap.size(); ++i) {
    vector<string> m = strSplit(typesMap[i], "=");
    if (m.size() != 2) {
      cerr << __PRETTY_FUNCTION__ << ": unable to extract type map from " << typesMap[i] << endl;
      continue;
    }
    string typeInFile = trim(m[0]);
    string loadedType = trim(m[1]);
    if (! factory->knowsTag(loadedType)) {
      cerr << __PRETTY_FUNCTION__ << ": unknown type " << loadedType << endl;
      continue;
    }

    _renamedTypesLookup[typeInFile] = loadedType;
  }

  cerr << "# load look up table" << endl;
  for (std::map<std::string, std::string>::const_iterator it = _renamedTypesLookup.begin(); it != _renamedTypesLookup.end(); ++it) {
    cerr << "#\t" << it->first << " -> " << it->second << endl;
  }
}

bool OptimizableGraph::isSolverSuitable(const SolverProperty& solverProperty, const std::set<int>& vertDims_) const
{
  std::set<int> auxDims;
  if (vertDims_.size() == 0) {
    auxDims = dimensions();
  }
  const set<int>& vertDims = vertDims_.size() == 0 ? auxDims : vertDims_;
  bool suitableSolver = true;
  if (vertDims.size() == 2) {
    if (solverProperty.requiresMarginalize) {
      suitableSolver = vertDims.count(solverProperty.poseDim) == 1 && vertDims.count(solverProperty.landmarkDim) == 1;
    }
    else {
      suitableSolver = solverProperty.poseDim == -1;
    }
  } else if (vertDims.size() == 1) {
    suitableSolver = vertDims.count(solverProperty.poseDim) == 1 || solverProperty.poseDim == -1;
  } else {
    suitableSolver = solverProperty.poseDim == -1 && !solverProperty.requiresMarginalize;
  }
  return suitableSolver;
}

std::set<int> OptimizableGraph::dimensions() const
{
  std::set<int> auxDims;
  for (VertexIDMap::const_iterator it = vertices().begin(); it != vertices().end(); ++it) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
    auxDims.insert(v->dimension());
  }
  return auxDims;
}

void OptimizableGraph::preIteration(int iter)
{
  HyperGraphActionSet& actions = _graphActions[AT_PREITERATION];
  if (actions.size() > 0) {
    HyperGraphAction::ParametersIteration params(iter);
    for (HyperGraphActionSet::iterator it = actions.begin(); it != actions.end(); ++it) {
      (*(*it))(this, &params);
    }
  }
}

void OptimizableGraph::postIteration(int iter)
{
  HyperGraphActionSet& actions = _graphActions[AT_POSTITERATION];
  if (actions.size() > 0) {
    HyperGraphAction::ParametersIteration params(iter);
    for (HyperGraphActionSet::iterator it = actions.begin(); it != actions.end(); ++it) {
      (*(*it))(this, &params);
    }
  }
}

bool OptimizableGraph::addPostIterationAction(HyperGraphAction* action)
{
  std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_POSTITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::addPreIterationAction(HyperGraphAction* action)
{
  std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_PREITERATION].insert(action);
  return insertResult.second;
}

bool OptimizableGraph::removePreIterationAction(HyperGraphAction* action)
{
  return _graphActions[AT_PREITERATION].erase(action) > 0;
}

bool OptimizableGraph::removePostIterationAction(HyperGraphAction* action)
{
  return _graphActions[AT_POSTITERATION].erase(action) > 0;
}

bool OptimizableGraph::saveVertex(std::ostream& os, OptimizableGraph::Vertex* v) const
{
  Factory* factory = Factory::instance();
  string tag = factory->tag(v);
  if (tag.size() > 0) {
    os << tag << " " << v->id() << " ";
    v->write(os);
    os << endl;
    if (v->userData()) { // write the data packet for the vertex
      tag = factory->tag(v->userData());
      if (tag.size() > 0) {
        os << tag << " ";
        v->userData()->write(os);
        os << endl;
      }
    }
    return os.good();
  }
  return false;
}

bool OptimizableGraph::saveEdge(std::ostream& os, OptimizableGraph::Edge* e) const
{
  Factory* factory = Factory::instance();
  string tag = factory->tag(e);
  if (tag.size() > 0) {
    os << tag << " ";
    if (_edge_has_id)
      os << e->id() << " ";
    for (vector<HyperGraph::Vertex*>::const_iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      os << v->id() << " ";
    }
    e->write(os);
    os << endl;
    return os.good();
  }
  return false;
}

} // end namespace
