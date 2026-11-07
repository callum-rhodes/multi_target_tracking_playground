#pragma once
#include <vector>
#include "sensors/Measurement.hpp"
#include "core/Target.hpp"
#include "tracking/IFusion.hpp"

namespace sfsim {

// Everything the view needs; plain data, copied out of the sim each frame
// (or double-buffered if the sim moves to its own thread).
struct Snapshot {
  double time = 0;
  std::vector<std::pair<TargetId, TargetState>> truth;
  std::vector<Measurement> measurements;  // this tick only
  std::vector<Track> tracks;
  std::vector<Track> unfusedTracks;  // pre-fusion confirmed tracks (decentralised only)
};

}  // namespace sfsim
