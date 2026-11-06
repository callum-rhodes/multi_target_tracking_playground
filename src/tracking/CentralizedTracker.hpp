#pragma once
#include <vector>
#include "tracking/ExtendedKalmanFilter.hpp"
#include "tracking/IFusion.hpp"
#include "tracking/TrackerParams.hpp"

namespace sfsim {

// Fusion centre: all sensors' measurements -> one track pool.
// Association is greedy nearest-neighbour for now; replace with GNN (Hungarian) / MHT.
//
// One scan is processed as a pipeline:
//
//   beginScan -> [ associate / spawn phases, in any order you like ] -> endScan
//
// Every association phase takes the measurements it is offered BY VALUE and returns the ones
// it did not claim, so the output of one phase feeds the next and nothing needs a shared
// "used" flag. The per-track "was I updated this scan?" state lives inside the track pool
// (reset in beginScan, read in endScan). step() (the IFusion entry point) simply runs the
// centralised order: confirmed, tentative, spawn.
class CentralizedTracker : public IFusion {
 public:
  explicit CentralizedTracker(const TrackerParams& p) : params_(p) {}
  CentralizedTracker(TrackerParams&&) = delete;  // would dangle immediately
  void step(double t, TrackerId t_id, const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms) override;
  std::vector<Track> tracks() const override;

  // ---- the phases -------------------------------------------------------------------------
  // Predict every track to time t and decide, ONCE, whether a scan happened. `scanned` must be
  // computed from the original input: later phases may see an empty remainder simply because
  // an earlier phase claimed everything, which is not the same as "no scan".
  void beginScan(double t, TrackerId t_id, bool scanned);

  // Greedy NN between the offered measurements and the tracks that are Confirmed right now.
  std::vector<Measurement> associateConfirmed(const std::vector<Sensor>& sensors, std::vector<Measurement> ms);

  // Test the offered measurements against waiting priors (neighbour cues). A candidate that
  // collects enough hits is promoted into the pool as a Confirmed track. Updates the
  // priors' hit/miss counters in place; does not add or remove priors.
  std::vector<Measurement> associatePriors(const std::vector<Sensor>& sensors, std::vector<Measurement> ms,
                                               std::vector<trackPrior>& priors);

  // Greedy NN against the tracks that are still tentative at the moment this phase starts.
  std::vector<Measurement> associateTentative(const std::vector<Sensor>& sensors, std::vector<Measurement> ms);

  // Final phase: whatever is left starts new tentative tracks. Nothing is returned, so it takes
  // a const reference rather than ownership.
  void spawnTentatives(const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms);

  // Count misses, delete dead tracks, prune duplicates. Does nothing if no scan happened.
  void endScan();

 private:
  struct Internal {
    Track trackInfo;
    EKF kf;
    uint64_t sensors_used = 0;  // bit k set = sensor k already updated this track in the current scan
  };

  // Shared greedy-NN loop for the two track-based phases. Eligibility (confirmed vs tentative) is
  // snapshotted at the start so a tentative that gets confirmed mid-phase keeps taking
  // measurements from other sensors in the same scan, as the old single loop did.
  std::vector<Measurement> associate(const std::vector<Sensor>& sensors, std::vector<Measurement> ms, bool confirmed);

  // Build a confirmed local track from a candidate's aged estimate and the measurement that promoted it.
  void promote(trackPrior& c, const EKF& aged, const Measurement& m, const Sensor& s);

  const TrackerParams& params_;
  std::vector<Internal> pool_;
  uint32_t next_id_ = 1;
  double last_t_ = -1;
  TrackerId t_id_ = 0;   // stamped onto tracks this tracker creates
  bool scanned_ = false; // did a scan happen this tick? (set in beginScan)
};

}  // namespace sfsim
