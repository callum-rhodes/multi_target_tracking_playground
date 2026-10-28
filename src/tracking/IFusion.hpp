#pragma once
#include <vector>
#include "sensors/Measurement.hpp"
#include "sensors/Sensor.hpp"

namespace sfsim {

struct Track {
  uint32_t id = 0;
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();
  int hits = 0, misses = 0;
  bool confirmed = false;
};

// The seam between layers B and C. Centralized and decentralized
// architectures both implement this, so the sim loop and GUI never care which is active.
class IFusion {
 public:
  virtual ~IFusion() = default;
  // `measurements` = everything delivered this tick (already noise-corrupted).
  virtual void step(double t, const std::vector<Sensor>& sensors, const std::vector<Measurement>& measurements) = 0;
  virtual std::vector<Track> tracks() const = 0;
};

}  // namespace sfsim
