#pragma once
#include "core/Types.hpp"
#include "sensors/Measurement.hpp"

namespace sfsim {

class Sensor;  // full definition only needed in the .cpp

// Constant-velocity extended Kalman filter, state [x y z vx vy vz].
// The measurement equations (h, H, residual) come from the sensor's own
// MeasurementModel, so the filter handles any registered MeasType without branching.
struct EKF {
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();

  void predict(double dt, double q_accel);
  // Returns the Mahalanobis distance squared of the innovation.
  // `sensor` must be the sensor that produced `m`.
  double gateDistance(const Measurement& m, const Sensor& sensor) const;
  void update(const Measurement& m, const Sensor& sensor);
};

}  // namespace sfsim
