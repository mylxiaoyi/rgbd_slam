#ifndef SCLAM_HELPERS_H
#define SCLAM_HELPERS_H

namespace g2o {

  struct SparseOptimizer;
  class DataQueue;

  void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer, const DataQueue& odomData);

  void allocateSolverForSclam(SparseOptimizer& optimizer);

} // end namespace

#endif
