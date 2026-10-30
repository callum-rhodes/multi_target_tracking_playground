#pragma once
#include <vector>
#include "sensors/Measurement.hpp"
#include "sensors/Sensor.hpp"

namespace sfsim {
enum class TrackStatus { Unconfirmed, Confirmed, Fused };

// Tracks are only unique per node, so anything that holds tracks from several nodes keys by (node, id).
using TrackKey = uint64_t;
constexpr TrackKey trackKey(TrackerId node, uint32_t id) {
  return (TrackKey{node} << 32) | id;
}

struct TrackFusionRef {            // identifies one track in the system
  TrackerId t_id = 0;   // which node fused from
  uint32_t id = 0;      // that node's local track id
};

struct Track {
  uint32_t tracker_id = 0; // TrackerId of the tracker that created this track (for decentralized fusion)
  uint32_t id = 0;
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();
  int hits = 0, misses = 0;
  TrackStatus status = TrackStatus::Unconfirmed;
  std::vector<TrackFusionRef> fused_from;  // which tracks were fused to make this one
};

// The seam between layers B and C. Centralized and decentralized
// architectures both implement this, so the sim loop and GUI never care which is active.
class IFusion {
 public:
  virtual ~IFusion() = default;
  // `measurements` = everything delivered this tick (already noise-corrupted).
  virtual void step(double t, TrackerId t_id, const std::vector<Sensor>& sensors, const std::vector<Measurement>& measurements) = 0;
  virtual std::vector<Track> tracks() const = 0;
};

}  // namespace sfsim
