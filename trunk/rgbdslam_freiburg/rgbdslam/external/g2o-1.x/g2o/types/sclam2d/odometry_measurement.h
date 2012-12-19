#ifndef ODOMETRY_MEASUREMENT_H
#define ODOMETRY_MEASUREMENT_H

#include <Eigen/Core>

namespace g2o {

  /**
   * \brief velocity measurement of a differential robot
   */
  class VelocityMeasurement
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VelocityMeasurement();
      VelocityMeasurement(double vl, double vr, double dt);

      const double& vl() const { return _measurement(0);}
      double& vl() { return _measurement(0);}

      const double& vr() const { return _measurement(1);}
      double& vr() { return _measurement(1);}

      const double& dt() const { return _measurement(2);}
      double& dt() { return _measurement(2);}
      
      const Eigen::Vector3d& measurement() const { return _measurement;}

    protected:
      Eigen::Vector3d _measurement;
  };

  /**
   * \brief A 2D odometry measurement expressed as a transformation
   */
  class MotionMeasurement
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      MotionMeasurement();
      MotionMeasurement(double x, double y, double theta, double dt);
      MotionMeasurement(const Eigen::Vector3d& m, double dt);

      const double& x() const { return _measurement(0);}
      double& x() { return _measurement(0);}

      const double& y() const { return _measurement(1);}
      double& y() { return _measurement(1);}

      const double& theta() const { return _measurement(2);}
      double& theta() { return _measurement(2);}

      const double& dt() const { return _dt;}
      double& dt() { return _dt;}

      const Eigen::Vector3d& measurement() const { return _measurement;}

    protected:
      Eigen::Vector3d _measurement;
      double _dt;
  };

  /**
   * \brief convert between the different types of odometry measurements
   */
  class OdomConvert
  {
    public:
      static VelocityMeasurement convertToVelocity(const MotionMeasurement& m);
      static MotionMeasurement convertToMotion(const VelocityMeasurement& vi, double l = 1.0);
  };

} // end namespace

#endif
