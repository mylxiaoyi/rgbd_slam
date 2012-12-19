#include "odometry_measurement.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {

  VelocityMeasurement::VelocityMeasurement() :
    _measurement(0., 0., 0.)
  {
  }

  VelocityMeasurement::VelocityMeasurement(double vl, double vr, double dt) :
    _measurement(vl, vr, dt)
  {
  }

  MotionMeasurement::MotionMeasurement() :
    _measurement(0., 0., 0.), _dt(0.)
  {
  }

  MotionMeasurement::MotionMeasurement(double x, double y, double theta, double dt) :
    _measurement(x, y, theta), _dt(dt)
  {
  }

  MotionMeasurement::MotionMeasurement(const Eigen::Vector3d& m, double dt) :
    _measurement(m), _dt(dt)
  {
  }

  VelocityMeasurement OdomConvert::convertToVelocity(const MotionMeasurement& m)
  {
    Eigen::Vector2d px2(0, 10);

    if (fabs(m.theta()) > 1e-7) {
      Eigen::Rotation2Dd rot(m.theta());
      Eigen::Vector2d px3(m.x(), m.y());
      Eigen::Vector2d px4(rot * px2 + px3);

      const double& y2 = px2.y();
      const double& x3 = px3.x();
      const double& y3 = px3.y();
      const double& x4 = px4.x();
      const double& y4 = px4.y();

      double R = (y2 * (x3*y4 - y3*x4)) / (y2 * (x3 - x4));
      double w;
      if (fabs(m.dt()) > 1e-7)
        w = m.theta() / m.dt();
      else
        w = 0.;

      double vl = (2.*R*w - w) / 2.;
      double vr = w + vl;

      return VelocityMeasurement(vl, vr, m.dt());
    } else {
      VelocityMeasurement result;
      result.dt() = m.dt();
      if (fabs(result.dt()) > 1e-7)
        result.vl() = result.vr() = hypot(m.x(), m.y()) / result.dt();
      else
        result.vl() = result.vr() = 0.;
      return result;
    }
  }

  MotionMeasurement OdomConvert::convertToMotion(const VelocityMeasurement& v, double l)
  {
    MotionMeasurement result;
    result.dt() = v.dt();

    if (fabs(v.vr() - v.vl()) > 1e-7) {

      double R = l * 0.5 * ((v.vl() + v.vr())  / (v.vr() - v.vl()));
      double w = (v.vr() - v.vl()) / l;

      result.theta() = w * v.dt();
      Eigen::Rotation2Dd rot(result.theta());
      Eigen::Vector2d icc(0, R);
      Eigen::Vector2d motion = (rot * (Eigen::Vector2d(-1.*icc))) + icc;
      result.x() = motion.x();
      result.y() = motion.y();

    } else {

      double tv = 0.5 * (v.vr() + v.vl());
      result.theta() = 0.;
      result.x() = tv * v.dt();
      result.y() = 0.;

    }

    return result;
  }

} // end namespace
