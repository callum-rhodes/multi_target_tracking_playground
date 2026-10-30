#pragma once
#include "tracking/IFusion.hpp"
#include "tracking/ExtendedKalmanFilter.hpp"
#include "tracking/TrackerParams.hpp"

namespace sfsim {

// Fusion centre: all sensors' measurements -> one track pool.
// Association is greedy nearest-neighbour for now; replace with GNN (Hungarian) / MHT.
class CentralizedTracker : public IFusion {
 public:
explicit CentralizedTracker(const TrackerParams& p) : p_(p) {}
  CentralizedTracker(TrackerParams&&) = delete;  // would dangle immediately
  void step(double t, TrackerId t_id, const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms) override;
  std::vector<Track> tracks() const override;

 private:
  struct Internal { Track meta; EKF kf; };
  const TrackerParams& p_;
  std::vector<Internal> pool_;
  uint32_t next_id_ = 1;
  double last_t_ = -1;
};

}  // namespace sfsim
