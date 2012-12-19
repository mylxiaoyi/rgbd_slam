// This example consists of a single static target which sits in one
// place and does not move; in effect it has a "GPS" which measures
// its position

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
 
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "samplers.hpp"

#include "targetTypes3D.hpp"

using namespace Eigen;
using namespace std;
using namespace g2o;


int main()
{
  // Set up the optimiser
  SparseOptimizer optimizer;
  optimizer.setMethod(SparseOptimizer::GaussNewton);
  optimizer.setVerbose(false);

  // Create the block solver - the dimensions are specified because
  // 3D observations marginalise to a 3D estimate
  typedef BlockSolver< BlockSolverTraits<3, 3> > BlockSolver_3_3;
  BlockSolver_3_3::LinearSolverType * linearSolver
      = new LinearSolverCholmod<BlockSolver_3_3::PoseMatrixType>();
  BlockSolver_3_3 * solver_ptr
      = new BlockSolver_3_3(&optimizer,linearSolver);

  optimizer.setSolver(solver_ptr);

  // giving some seed
  Sample::seed();

  // Sample the actual location of the target
  Vector3d truePoint((Sample::uniform()-0.5)*1000,
                     (Sample::uniform()-0.5)*1000,
                     (Sample::uniform()-0.5)*1000);

  // Construct vertex which corresponds to the actual point of the target
  VertexPosition3D* position = new VertexPosition3D();
  position->setId(0);
  position->setMarginalized(false);
  optimizer.addVertex(position);

  // Now generate some noise corrupted measurements; for simplicity
  // these are uniformly distributed about the true target. These are
  // modelled as a unary edge because they do not like to, say,
  // another node in the map.
  int numMeasurements = 100;
  double noiseLimit = 500;
  double noiseSigma = noiseLimit*noiseLimit / 12.0;

  for (int i = 0; i < numMeasurements; i++)
    {
      Vector3d measurement = truePoint +
        Vector3d((Sample::uniform()-0.5) * noiseLimit,
                 (Sample::uniform()-0.5) * noiseLimit,
                 (Sample::uniform()-0.5) * noiseLimit);
      GPSObservationPosition3DEdge* goe = new GPSObservationPosition3DEdge();
      goe->vertices()[0] = position;
      goe->setMeasurement(measurement);
      goe->information() = Matrix3d::Identity() / noiseSigma;
      goe->setRobustKernel(false);
      optimizer.addEdge(goe);
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(1);
  
  cout << "truePoint=\n" << truePoint << endl;

  cerr <<  "computed estimate=\n" << dynamic_cast<VertexPosition3D*>(optimizer.vertices().find(0)->second)->estimate() << endl;

  // covariance
  optimizer.solver()->computeMarginals();
  cerr << "covariance\n" << position->uncertainty() << endl;
}
