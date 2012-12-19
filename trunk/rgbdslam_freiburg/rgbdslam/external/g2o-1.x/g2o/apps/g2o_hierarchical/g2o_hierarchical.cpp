// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <signal.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <sstream>
//#include <Eigen/Dense>
//#include "unscented.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/output_helper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/factory.h"
#include "g2o/core/solver_factory.h"
#include "g2o/core/hyper_dijkstra.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "edge_labeler.h"
#include "edge_creator.h"
#include "edge_types_cost_function.h"
#include "star.h"
//#include "backbone_tree_action.h"
#include "simple_star_ops.h"


#include "g2o/types/slam3d/camera_parameters.h"
#include "g2o/types/slam3d/se3_offset_parameters.h"

static bool hasToStop=false;

using namespace std;
using namespace g2o;

SparseOptimizer::Method str2method(const std::string& strMethod_){
  string strMethod = strToLower(strMethod_);
  if (strMethod=="gauss") {
    cerr << "# Doing Gauss" << endl;
    return SparseOptimizer::GaussNewton;
  }
  if (strMethod=="levenberg") {
    cerr << "# Doing Levenberg-Marquardt" << endl;
    return SparseOptimizer::LevenbergMarquardt;
  }
  cerr << "# Unknown optimization method: " << strMethod << ", setting  default to Levenberg"  << endl;
  return SparseOptimizer::LevenbergMarquardt;
}

void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

int main(int argc, char** argv)
{

  int maxIterations;
  bool verbose;
  string inputFilename;
  string gnudump;
  string outputfilename;
  string strMethod;
  string strSolver;
  string loadLookup;
  string hierarchicalMode;
  bool initialGuess;
  bool marginalize;
  bool listTypes;
  bool listSolvers;
  bool guiOut;
  bool robustKernel;
  bool computeMarginals;
  double huberWidth;
  double lambdaInit;
  int hierarchicalDiameter;
  int updateGraphEachN = 10;
  string statsFile;
  string dummy;
  // command line parsing
  CommandArgs arg;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("hierarchicalMode", hierarchicalMode, "2d", "selects the type of hierarchy to construct {2d,3d,depth}");
  arg.param("hierarchicalDiameter", hierarchicalDiameter, -1 , "selects the diameter of the stars in the hierarchical graph");
  arg.param("guess", initialGuess, false, "initial guess based on spanning tree");
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes, (default: 10)");
  arg.param("guiout", guiOut, false, "gui output while running incrementally");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("marginalize", marginalize, false, "on or off");
  arg.param("method", strMethod, "Gauss", "Gauss or Levenberg");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("computeMarginals", computeMarginals, false, "computes the marginal covariances of something. FOR TESTING ONLY");
  arg.param("huberWidth", huberWidth, -1., "width for the robust Huber Kernel (only if robustKernel)");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "var", "specify which solver to use underneat\n\t {var, fix3_2, fix6_3, fix_7_3}");
  arg.param("solverlib", dummy, "", "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "", "specify a types library which will be loaded");
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("listSolvers", listSolvers, false, "list the available solvers");
  arg.param("renameTypes", loadLookup, "", "create a lookup for loading types into other types,\n\t TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  SolverFactory* solverFactory = SolverFactory::instance();
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);
  if (listSolvers)
    solverFactory->listSolvers(cerr);

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(cout, true);
  }

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

  // Loading the input data
  if (loadLookup.size() > 0) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.size() == 0) {
    cerr << "No input data specified" << endl;
    return 0;
  } else if (inputFilename == "-") {
    cerr << "Read input from stdin" << endl;
    if (!optimizer.load(cin)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  } else {
    cerr << "Read input from " << inputFilename << endl;
    ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      cerr << "Failed to open file" << endl;
      return 1;
    }
    if (!optimizer.load(ifs)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  }
  cerr << "Loaded " << optimizer.vertices().size() << " vertices" << endl;
  cerr << "Loaded " << optimizer.edges().size() << " edges" << endl;


  EdgeCreator creator;
  creator.addAssociation("VERTEX_SE2;VERTEX_SE2;","EDGE_SE2");
  creator.addAssociation("VERTEX_SE2;VERTEX_XY;","EDGE_SE2_XY");
  creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_SE3:QUAT;","EDGE_SE3:QUAT");
  OptimizableGraph::Parameters* p0 = optimizer.parameters(0);
  if (p0){
    CameraParameters* originalParams = dynamic_cast<CameraParameters*>(p0);
    if (p0) {
      std::vector<int> depthCamHParamsIds(1);
      depthCamHParamsIds[0]=0;
      creator.addAssociation("VERTEX_SE3:QUAT;VERTEX_TRACKXYZ;","EDGE_SE3_TRACKXYZ",depthCamHParamsIds);    
    }
  }

  EdgeLabeler labeler(&optimizer);

  if (optimizer.vertices().size() == 0) {
    cerr << "Graph contains no vertices" << endl;
    return 1;
  }

  // allocating the desired solver + testing whether the solver is okay
  SolverProperty solverProperty;
  optimizer.setSolver(solverFactory->construct(strSolver, &optimizer, solverProperty));
  if (! optimizer.solver()) {
    cerr << "Error allocating solver. Allocating \"" << strSolver << "\" failed!" << endl;
    return 0;
  }
  set<int> vertexDimensions = optimizer.dimensions();
  if (! optimizer.isSolverSuitable(solverProperty, vertexDimensions)) {
    cerr << "The selected solver is not suitable for optimizing the given graph" << endl;
    return 3;
  }
  assert (optimizer.solver());
  optimizer.setMethod(str2method(strMethod));
  optimizer.setUserLambdaInit(lambdaInit);


  // here we need to chop the graph into many lil pieces
  

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = optimizer.gaugeFreedom();
  OptimizableGraph::Vertex* gauge=optimizer.findGauge();

  
  
  
  if (gaugeFreedom) {
    if (! gauge) {
      cerr <<  "# cannot find a vertex to fix in this thing" << endl;
      return 2;
    } else {
      cerr << "# graph is fixed by node " << gauge->id() << endl;
      gauge->setFixed(true);
    }
  } else {
    cerr << "# graph is fixed by priors" << endl;
  }


  

  // if schur, we wanna marginalize the landmarks...
  if (marginalize || solverProperty.requiresMarginalize) {
    int maxDim = *vertexDimensions.rbegin();
    int minDim = *vertexDimensions.begin();
    if (maxDim != minDim) {
      cerr << "# Preparing Marginalization of the Landmarks ... ";
      for (HyperGraph::VertexIDMap::iterator it=optimizer.vertices().begin(); it!=optimizer.vertices().end(); it++){
        OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
        if (v->dimension() != maxDim) {
          v->setMarginalized(true);
        }
      }
      cerr << "done." << endl;
    }
  }

  if (robustKernel) {
    cerr << "# Preparing robust error function ... ";
    for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
      e->setRobustKernel(true);
      if (huberWidth > 0)
        e->setHuberWidth(huberWidth);
    }
    cerr << "done." << endl;
  }

  // sanity check
  HyperDijkstra d(&optimizer);
  UniformCostFunction f;
  d.shortestPaths(gauge,&f);
  //cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size()!=optimizer.vertices().size()) {
    cerr << CL_RED("Warning: d.visited().size() != optimizer.vertices().size()") << endl;
    cerr << "visited: " << d.visited().size() << endl;
    cerr << "vertices: " << optimizer.vertices().size() << endl;
  }


  // BATCH optimization

  if (statsFile!=""){
    // allocate buffer for statistics;
    optimizer._statistics=new G2OBatchStatistics[maxIterations];
  }
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cerr << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  if (initialGuess)
    optimizer.computeInitialGuess();
  signal(SIGINT, sigquit_handler);

  StarSet stars;
  if (hierarchicalMode == "3d") {
    hierarchicalDiameter = (hierarchicalDiameter == -1) ? 4 : hierarchicalDiameter;
    computeSimpleStars(stars, &optimizer, &labeler, &creator, 
    			gauge, "EDGE_SE3:QUAT", "VERTEX_SE3:QUAT", 0, hierarchicalDiameter, 
    			1, maxIterations);
    // computeSimpleStars(stars, &optimizer, &labeler, &creator, 
    // 			gauge, "EDGE_SE3_OFFSET", "VERTEX_SE3:QUAT", 0, hierarchicalDiameter, 
    // 			1, maxIterations);

  } else if (hierarchicalMode == "2d") {
    hierarchicalDiameter = (hierarchicalDiameter == -1) ? 30 : hierarchicalDiameter;
    computeSimpleStars(stars, &optimizer, &labeler, &creator, 
    			gauge, "EDGE_SE2", "VERTEX_SE2", 0, hierarchicalDiameter,
			1, maxIterations );

  } 

  cerr << "stars computed, stars.size()= " << stars.size() << endl;
 
  cerr << "hierarchy done, determining border" << endl;
  EdgeStarMap hesmap;
  constructEdgeStarMap(hesmap, stars, false);
  computeBorder(stars, hesmap);
  
  OptimizableGraph::EdgeSet eset;
  OptimizableGraph::EdgeSet heset;

  for (StarSet::iterator it=stars.begin(); it!=stars.end(); it++){
    Star* s=*it;
    for (HyperGraph::EdgeSet::iterator iit=s->_starEdges.begin(); iit!=s->_starEdges.end(); iit++){
      OptimizableGraph::Edge* e= (OptimizableGraph::Edge*) *iit;
      eset.insert(e);
    }
    for (HyperGraph::EdgeSet::iterator iit=s->starFrontierEdges().begin(); iit!=s->starFrontierEdges().end(); iit++){
      OptimizableGraph::Edge* e= (OptimizableGraph::Edge*) *iit;
      heset.insert(e);
    }
  }
  cerr << "eset.size()= " << eset.size() << endl;
  cerr << "heset.size()= " << heset.size() << endl;
  
  ofstream starStream("stars.g2o");
  optimizer.saveSubset(starStream, eset);
  starStream.close();

  ofstream hstarStream("hstars.g2o");
  optimizer.saveSubset(hstarStream, heset);
  hstarStream.close();

  return 0;


  int i=optimizer.optimize(maxIterations);
  if (maxIterations > 0 && !i){
    cerr << "Cholesky failed, result might be invalid" << endl;
  } else {
    cerr << "attempting to re-label the edges from the gauge" << endl;
    std::set<OptimizableGraph::Edge*> eset;
    for (HyperGraph::EdgeSet::iterator it=optimizer.edges().begin(); it!=optimizer.edges().end(); it++){
      eset.insert((OptimizableGraph::Edge*)*it);
    }
    EdgeLabeler labeler(&optimizer);
    labeler.labelEdges(eset);
  }

  if (statsFile!=""){
    cerr << "writing stats to file \"" << statsFile << "\" ... ";
    ofstream os(statsFile.c_str());
    for (int i=0; i<maxIterations; i++){
      os << optimizer._statistics[i] << endl;
    }
    cerr << "done." << endl;
  }


  // saving again
  if (gnudump.size() > 0) {
    return saveGnuplot(gnudump, optimizer);
  }

  if (outputfilename.size() > 0) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      optimizer.save(cout);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      optimizer.save(outputfilename.c_str());
    }
    cerr << "done." << endl;
  }


  return 0;
}
