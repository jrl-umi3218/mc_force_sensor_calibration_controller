#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <map>
#include <string>
#include <vector>

/** A measurement for the calibration */
struct Measurement
{
  /** Position of the force sensor's parent body */
  sva::PTransformd X_0_p;
  /** Raw force sensor reading */
  sva::ForceVecd measure;
};

using Measurements = std::vector<Measurement>;
using SensorMeasurements = std::map<std::string, Measurements>;
