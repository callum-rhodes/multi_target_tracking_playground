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
  uint32_t tracker_id = 0; // TrackerId of the tracker that created this track (for decentralised fusion)
  uint32_t id = 0;
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();
  int hits = 0, misses = 0;
  TrackStatus status = TrackStatus::Unconfirmed;
  std::vector<TrackFusionRef> fused_from;  // which tracks were fused to make this one
};

// Track information for a prior track that is not locally fused but may inform the local tracker for new spawn candidates.
struct trackPrior {
  TrackFusionRef source;       // (neighbour node, neighbour's local track id): the cue's identity
  Vec6 x = Vec6::Zero();       // prior state estimate ...
  Mat6 P = Mat6::Identity();   // ... and covariance, both valid at `stamp`
  double stamp = 0;            // time of the prior estimate generation; also the freshness clock
  int hits = 0;                // own-sensor hits (consecutive: see prior_reset_misses)
  int misses = 0;              // consecutive scans without a hit
  bool promoted = false;       // already turned into a local track
};

// The seam between layers B and C. centralised and decentralised
// architectures both implement this, so the sim loop and GUI never care which is active.
class IFusion {
 public:
  virtual ~IFusion() = default;
  // `measurements` = everything delivered this tick (already noise-corrupted).
  virtual void step(double t, TrackerId t_id, const std::vector<Sensor>& sensors, const std::vector<Measurement>& measurements) = 0;
  virtual std::vector<Track> tracks() const = 0;

  // Confirmed tracks as each node holds them BEFORE track-to-track fusion.
  virtual std::vector<Track> preFusionTracks() const { return {}; }
};

}  // namespace sfsim
