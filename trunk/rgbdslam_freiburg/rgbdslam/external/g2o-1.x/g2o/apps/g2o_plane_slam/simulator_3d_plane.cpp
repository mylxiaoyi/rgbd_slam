#include <fstream>
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/types/slam3d/types_six_dof_quat.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"

#include <iostream>

using namespace g2o;
using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6,6> Matrix6d;

double uniform_rand(double lowerBndr, double upperBndr)
{
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

double gauss_rand(double sigma)
{
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

SE3Quat sample_noise_from_se3(const Vector6d cov ){
  double nx=gauss_rand(cov(0));
  double ny=gauss_rand(cov(1));
  double nz=gauss_rand(cov(2));

  double nyaw=gauss_rand(cov(3));
  double nroll=gauss_rand(cov(4));
  double npitch=gauss_rand(cov(5));

  AngleAxisd aa(
    AngleAxisd(nyaw, Vector3d::UnitZ())*
    AngleAxisd(nroll, Vector3d::UnitX())*
    AngleAxisd(npitch, Vector3d::UnitY()));
  SE3Quat q(aa.toRotationMatrix(),Vector3d(nx,ny,nz));
  return q;
}

Vector3d sample_noise_from_plane(const Vector3d& cov ){
  return Vector3d(gauss_rand(cov(0)), gauss_rand(cov(1)), gauss_rand(cov(2)));
}

struct SimulatorItem{
  SimulatorItem(OptimizableGraph* graph_): _graph(graph_){}
  OptimizableGraph* graph() {return _graph;}
  virtual ~SimulatorItem(){}
protected:
  OptimizableGraph* _graph;
};

struct WorldItem: public SimulatorItem {
  WorldItem(OptimizableGraph* graph_, OptimizableGraph::Vertex* vertex_ = 0) : 
    SimulatorItem(graph_),_vertex(vertex_) {} 
  OptimizableGraph::Vertex* vertex() {return _vertex;}
  void  setVertex(OptimizableGraph::Vertex* vertex_) {_vertex = vertex_;}
protected:
  OptimizableGraph::Vertex* _vertex;
};

typedef std::set<WorldItem*> WorldItemSet;

struct Robot;

struct Sensor {
  Sensor(Robot* robot_) : _robot(robot_){}
  Robot* robot() {return _robot;}
  virtual bool isVisible(const WorldItem* ) const {return false;}
  virtual bool sense(WorldItem* , const SE3Quat& ) {return false;}
  virtual ~Sensor(){};
protected:
  Robot* _robot;
};

typedef std::vector<Sensor*> SensorVector;

struct Robot: public WorldItem {

  Robot(OptimizableGraph* graph_): WorldItem(graph_) {
    _planarMotion=false;
  }
  

  void move(const SE3Quat& newPosition, int& id) {
    SE3Quat delta = _position.inverse()*newPosition;
    _position = newPosition;
    VertexSE3* v=new VertexSE3();
    v->setId(id);
    id++;
    graph()->addVertex(v);
    if (_planarMotion){
      // add a singleton constraint that locks the position of the robot on the plane
      EdgeSE3Prior* planeConstraint=new EdgeSE3Prior();
      Matrix6d pinfo = Matrix6d::Zero();
      pinfo(2,2)=1e9;
      planeConstraint->setInformation(pinfo);
      planeConstraint->setMeasurement(SE3Quat());
      planeConstraint->vertices()[0]=v;
      planeConstraint->setCacheId(0,0);
      graph()->addEdge(planeConstraint);
    }
    if (vertex()){
      VertexSE3* oldV=dynamic_cast<VertexSE3*>(vertex());
      EdgeSE3* e=new EdgeSE3();
      SE3Quat noise=sample_noise_from_se3(_nmovecov);
      e->setMeasurement(delta*noise);
      Matrix6d m=Matrix6d::Identity();
      for (int i=0; i<4; i++){
	m(i,i)=1./(_nmovecov(i));
      }
      m(4,4)=m(5,5)=1e4;
      e->setInformation(m);
      e->vertices()[0]=vertex();
      e->vertices()[1]=v;
      graph()->addEdge(e);
      v->setEstimate(oldV->estimate()*e->measurement());
    } else {
      v->setEstimate(_position);
    }
    setVertex(v);
  }

  void relativeMove(const SE3Quat& delta, int& id){
    SE3Quat newPosition = _position*delta;
    move(newPosition, id);
  }

  void sense(WorldItem* wi=0){
    for (size_t i=0; i<_sensors.size(); i++){
      Sensor* s=_sensors[i];
      s->sense(wi, _position);
    }
  }

  SE3Quat _position;
  SensorVector _sensors; 
  Vector6d _nmovecov;
  bool _planarMotion;
};

typedef std::vector<Robot*> RobotVector;

struct Simulator: public SimulatorItem {
  Simulator(OptimizableGraph* graph_): SimulatorItem(graph_), _lastVertexId(0){}
  void sense(int robotIndex){
    Robot* r=_robots[robotIndex];
    for (WorldItemSet::iterator it=_world.begin(); it!=_world.end(); it++){
      WorldItem* item=*it;
      r->sense(item);
    }
  }

  void move(int robotIndex, const SE3Quat& newRobotPose){
    Robot* r=_robots[robotIndex];
    r->move(newRobotPose, _lastVertexId);
  }

  void relativeMove(int robotIndex, const SE3Quat& delta){
    Robot* r=_robots[robotIndex];
    r->relativeMove(delta, _lastVertexId);
  }

  int _lastVertexId;
  WorldItemSet _world;
  RobotVector _robots;
};

struct PlaneItem: public WorldItem{
  PlaneItem(OptimizableGraph* graph_, int id) : WorldItem(graph_){
    VertexPlane* p=new VertexPlane();
    p->setId(id);
    graph()->addVertex(p);
    setVertex(p);
  } 
};

struct PlaneSensor: public Sensor{
  PlaneSensor(Robot* r, int offsetId, const SE3Quat& offset_): Sensor(r){
    _offsetVertex = new VertexSE3();
    _offsetVertex->setId(offsetId);
    _offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(_offsetVertex);
  };
  
  virtual bool isVisible(WorldItem* wi){
    if (! wi)
      return false;
    PlaneItem* pi=dynamic_cast<PlaneItem*>(wi);
    if (! pi)
      return false;
    return true;
  }

  virtual bool sense(WorldItem* wi, const SE3Quat& position){
    if (! wi)
      return false;
    PlaneItem* pi=dynamic_cast<PlaneItem*>(wi);
    if (! pi)
      return false;
    OptimizableGraph::Vertex* rv = robot()->vertex();
    if (! rv) {
      return false;
    }
    VertexSE3* robotVertex = dynamic_cast<VertexSE3*>(rv);
    if (! robotVertex){
      return false;
    }
    SE3Quat robotPose=position;
    SE3Quat sensorPose=robotPose*_offsetVertex->estimate();
    VertexPlane* planeVertex=dynamic_cast<VertexPlane*>(pi->vertex());
    Plane3D worldPlane=planeVertex->estimate();
    
    Plane3D measuredPlane=sensorPose.inverse()*worldPlane;

    EdgeSE3PlaneSensorCalib* e=new EdgeSE3PlaneSensorCalib();
    e->vertices()[0]=robotVertex;
    e->vertices()[1]=planeVertex;
    e->vertices()[2]=_offsetVertex;
    Vector3d noise = sample_noise_from_plane(_nplane);
    measuredPlane.oplus(noise);
    e->setMeasurement(measuredPlane);
    Matrix3d m=Matrix3d::Zero();
    m(0,0)=1./(_nplane(0));
    m(1,1)=1./(_nplane(1));
    m(2,2)=1./(_nplane(2));
    e->setInformation(m);
    robot()->graph()->addEdge(e);
    return true;
  }
  
  VertexSE3* _offsetVertex;
  Vector3d _nplane;
};

int main (int argc  , char ** argv){
  int maxIterations;
  bool verbose;
  bool robustKernel;
  double lambdaInit;
  CommandArgs arg;
  bool fixSensor;
  bool fixPlanes;
  bool fixFirstPose;
  bool planarMotion;
  cerr << "graph" << endl;
  arg.param("i", maxIterations, 5, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("lambdaInit", lambdaInit, 0, "user specified lambda init for levenberg");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("fixSensor", fixSensor, false, "fix the sensor position on the robot");
  arg.param("fixFirstPose", fixFirstPose, false, "fix the first robot pose");
  arg.param("fixPlanes", fixPlanes, false, "fix the planes (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.parseArgs(argc, argv);



  SparseOptimizer* g=new SparseOptimizer();
  SE3OffsetParameters* odomOffset=new SE3OffsetParameters();
  odomOffset->setId(0);
  g->addParameters(odomOffset);

  g->setMethod(g2o::SparseOptimizer::LevenbergMarquardt); 
  g2o::BlockSolverX::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>(); // alternative: CHOLMOD
  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(g,linearSolver);
  g->setSolver(solver_ptr);


  cerr << "sim" << endl;
  Simulator* sim = new Simulator(g);

  cerr << "robot" << endl;
  Robot* r=new Robot(g);
  
  
  cerr << "planeSensor" << endl;
  Matrix3d R=Matrix3d::Identity();
  R << 
    0,  0,   1,
    -1,  0,  0,
    0, -1,   0;

  SE3Quat sensorPose(R, Vector3d(.1 , 0.0 , 0.3));
  PlaneSensor* ps = new PlaneSensor(r, 0, sensorPose);
  ps->_nplane << 0.01, 0.01, 0.001;
  r->_sensors.push_back(ps);
  sim->_robots.push_back(r);
  
  cerr  << "p1" << endl; 
  Plane3D plane;
  PlaneItem* pi =new PlaneItem(g,1);
  plane.fromVector(Eigen::Vector4d(0.,0.,1.,5.));
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);

  plane.fromVector(Eigen::Vector4d(1.,0.,0.,5.));
  pi =new PlaneItem(g,2);
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);

  cerr  << "p2" << endl; 
  pi =new PlaneItem(g,3);
  plane.fromVector(Eigen::Vector4d(0.,1.,0.,5.));
  static_cast<VertexPlane*>(pi->vertex())->setEstimate(plane);
  pi->vertex()->setFixed(fixPlanes);
  sim->_world.insert(pi);

  Quaterniond q;
  if (planarMotion) {
    r->_planarMotion = true;
    r->_nmovecov << 0.1, 0.005, 1e-9, 0.05, 0.001, 0.001;
    q = Quaterniond(AngleAxisd(0.4, Vector3d::UnitZ()).toRotationMatrix());
  } else {
    r->_planarMotion = false;
    r->_nmovecov << 0.1, 0.005, 1e-9, 0.05, 0.001, 0.001;
    q = Quaterniond((AngleAxisd(0.4, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitY())).toRotationMatrix());
  }
  Vector3d tr(0.2,0,0);
  SE3Quat delta(q, tr);

  sim->_lastVertexId=4;
  sim->move(0,SE3Quat());
  
  for (int i=0; i<1000; i++){
    cerr << "rm" << endl;
    sim->relativeMove(0,delta);
    cerr << "sense" << endl;
    sim->sense(0);
  }

  ofstream os("test_gt.g2o");
  g->save(os);

  if (fixSensor) {
    ps->_offsetVertex->setFixed(true);
  } else {
    Vector6d noffcov;
    noffcov << 0.1,0.1,0.1,0.2, 0.2, 0.2;
    ps->_offsetVertex->setEstimate(
				   ps->_offsetVertex->estimate() * 
				   sample_noise_from_se3(noffcov)
				   );
    ps->_offsetVertex->setFixed(false);
  }

  if (fixFirstPose){
    OptimizableGraph::Vertex* gauge = g->vertex(4);
    if (gauge)
      gauge->setFixed(true);
  } else {
    // multiply all vertices of the robot by this standard quantity
    Quaterniond q(AngleAxisd(1, Vector3d::UnitZ()).toRotationMatrix());
    Vector3d tr(1,1,0);
    SE3Quat delta(q, tr);
    for (size_t i=0; i< g->vertices().size(); i++){
      VertexSE3 *v = dynamic_cast<VertexSE3 *>(g->vertex(i));
      if (v && v->id()>0){
	v->setEstimate (v->estimate()*delta);
      }
    }
  }

  ofstream osp("test_preopt.g2o");
  g->save(osp);
  g->setMethod(SparseOptimizer::LevenbergMarquardt);
  g->initializeOptimization();
  g->setVerbose(verbose);
  g->optimize(maxIterations);

  ofstream os1("test_postOpt.g2o");
  g->save(os1);

}
