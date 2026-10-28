#pragma once
#include "core/Types.hpp"

namespace sfsim {
  enum class MeasType { Cartesian, RangeAzEl };

  struct Measurement {
    SensorId sensor;
    Pose pose; // pose of the sensor at measurement time, in world coordinates
    double time;
    MeasType type;
    Vec3 z;                      // Cartesian: [x y z]   RangeAzEl: [r az el] in the sensor frame
    Mat3 R;                      // covariance in the measurement's own space
    // ground truth for scoring only
    bool is_clutter; TargetId truth_id;
  };
}  // namespace sfsim
