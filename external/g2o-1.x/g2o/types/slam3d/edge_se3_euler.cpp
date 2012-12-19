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

#include "edge_se3_euler.h"
#include "g2o/core/factory.h"
#include <iostream>

namespace g2o {

  bool EdgeSE3Euler::read(std::istream& is)
  {
    Vector6d meas;
    for (int i=0; i<6; i++)
      is  >> meas[i];
    Vector3d translation(meas[0], meas[1], meas[2]);
    Quaterniond rotation = euler_to_quat(meas[5], meas[4], meas[3]);
    SE3Quat transf(rotation, translation);
    Matrix<double, 6, 6> infMatEuler;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> infMatEuler(i,j);
        if (i!=j)
          infMatEuler(j,i) = infMatEuler(i,j);
      }
    Matrix<double, 6, 6> J;
    jac_quat3_euler3(J, transf);
    Matrix<double, 6, 6> infMat = J.transpose() * infMatEuler * J;
    setMeasurement(transf);
    setInformation(infMat);
    return true;
  }

  bool EdgeSE3Euler::write(std::ostream& os) const
  {
    Vector6d meas;
    meas.head<3>()=measurement().translation();
    quat_to_euler(measurement().rotation(), meas(5), meas(4), meas(3));
    for (int i=0; i<6; i++)
      os << meas[i] << " ";
    
    Matrix<double, 6, 6> J;
    jac_quat3_euler3(J, measurement());
    //HACK: invert the jacobian to simulate the inverse derivative
    J=J.inverse();
    Matrix<double, 6, 6> infMatEuler = J.transpose()*information()*J;
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  infMatEuler(i,j);
      }
    return os.good();
  }


} // end namespace
