#include <mc_rbdyn/Robot.h>

#include "Measurement.h"

struct CalibrationResult
{
  bool success = false;
  double mass = 0;
  double rpy[3] = {0};
  double com[3] = {0};
  double offset[6] = {0};
};

CalibrationResult calibrate(const mc_rbdyn::Robot & robot,
                            const std::string & sensor,
                            const Measurements & measurements,
                            bool verbose = false);
