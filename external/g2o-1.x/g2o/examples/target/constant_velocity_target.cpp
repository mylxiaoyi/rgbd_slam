// This example consists of a single constant velocity target which
// moves under piecewise constant velocity in 3D. Its position is
// measured by an idealised GPS receiver.

// I'm experimenting with some probably very silly stuff about putting
// velocity in the state; I know I shouldn't do it, but I was curious
// to see what gets produced!

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
 
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "samplers.hpp"

#include "targetTypes6D.hpp"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main()
{
  // Set up the parameters of the simulation
  int numberOfTimeSteps = 1000;
  const double processNoiseSigma = 1;
  const double accelerometerNoiseSigma = 1;
  const double gpsNoiseSigma = 1;
  const double dt = 1;  

  // Set up the optimiser and block solver
  SparseOptimizer optimizer;
  optimizer.setMethod(SparseOptimizer::GaussNewton);
  optimizer.setVerbose(false);

  g2o::BlockSolver_6_3::LinearSolverType * linearSolver
      = new g2o::LinearSolverCholmod<g2o
        ::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3 * solver_ptr
      = new g2o::BlockSolver_6_3(&optimizer,linearSolver);
  optimizer.setSolver(solver_ptr);

  // Sample the start location of the target
  Vector6d state;
  state.setZero();
  for (int k = 0; k < 3; k++)
    {
      state[k] = Sample::gaussian(1000);
    }
  
  // Construct the first vertex; this corresponds to the initial
  // condition and register it with the optimiser
  VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
  stateNode->setEstimate(state);
  stateNode->setId(0);
  stateNode->setMarginalized(false);
  optimizer.addVertex(stateNode);

  // Set up last estimate
  VertexPositionVelocity3D* lastStateNode = stateNode;

  // Iterate over the simulation steps
  for (int k = 1; k <= numberOfTimeSteps; k++)
    {
      // Simulate the next step; update the state and compute the observation
      Vector3d processNoise(Sample::gaussian(processNoiseSigma),
                            Sample::gaussian(processNoiseSigma),
                            Sample::gaussian(processNoiseSigma));

      for (int m = 0; m < 3; m++)
        {
          state[m] += dt * (state[m+3] + 0.5 * dt * processNoise[m]);
        }

      for (int m = 0; m < 3; m++)
        {
          state[m+3] += dt * processNoise[m];
        }

      // Construct the accelerometer measurement
      Vector3d accelerometerMeasurement;
      for (int m = 0; m < 3; m++)
        {
          accelerometerMeasurement[m] = processNoise[m] + Sample::gaussian(accelerometerNoiseSigma);
        }

      // Construct the GPS observation
      Vector3d gpsMeasurement;     
      for (int m = 0; m < 3; m++)
        {
          gpsMeasurement[m] = state[m] + Sample::gaussian(gpsNoiseSigma);
        }

      // Construct vertex which corresponds to the current state of the target
      VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
      
      stateNode->setId(k);
      stateNode->setMarginalized(false);
      optimizer.addVertex(stateNode);

      TargetOdometry3DEdge* toe = new TargetOdometry3DEdge(dt, accelerometerNoiseSigma);
      toe->vertices()[0] = lastStateNode;
      toe->vertices()[1] = stateNode;
      VertexPositionVelocity3D* vPrev= dynamic_cast<VertexPositionVelocity3D*>(lastStateNode);
      VertexPositionVelocity3D* vCurr= dynamic_cast<VertexPositionVelocity3D*>(stateNode);
      toe->setMeasurement(accelerometerMeasurement);
      optimizer.addEdge(toe);
      
      // compute the initial guess via the odometry
      toe->initialEstimate(vPrev,vCurr);

      lastStateNode = stateNode;

      // Add the GPS observation
      GPSObservationEdgePositionVelocity3D* goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
      goe->vertices()[0] = stateNode;
      optimizer.addEdge(goe);
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);
  cerr << "number of vertices:" << optimizer.vertices().size() << endl;
  cerr << "number of edges:" << optimizer.edges().size() << endl;

  // Print the results

  cout << "state=\n" << state << endl;

#if 0
  for (int k = 0; k < numberOfTimeSteps; k++)
    {
      cout << "computed estimate " << k << "\n"
           << dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(k)->second)->estimate() << endl;
       }
#endif

  Vector6d v1 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(max(numberOfTimeSteps-2,0))->second)->estimate();
  Vector6d v2 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(max(numberOfTimeSteps-1,0))->second)->estimate();
  cout << "v1=\n" << v1 << endl;
  cout << "v2=\n" << v2 << endl;
  cout << "delta state=\n" << v2-v1 << endl;
}
