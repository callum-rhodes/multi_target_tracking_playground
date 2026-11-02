#include "tracking/CentralizedTracker.hpp"
#include <limits>

namespace sfsim {

void CentralizedTracker::step(double t, TrackerId t_id, const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms) {
  const double dt = (last_t_ < 0) ? 0.0 : t - last_t_;
  last_t_ = t;
  for (auto& tr : pool_) if (dt > 0) tr.kf.predict(dt, p_.q_accel);

  // No sensor scanned this tick: coast only. Misses are counted per scan, not per tick.
  if (ms.empty()) return;

  std::vector<bool> usedMeas(ms.size(), false);
  std::vector<uint64_t> usedSensors(pool_.size(), 0);  // bit k set = sensor k already fused into track i this cycle

  // Map a measurement's sensor id to a bit. Assumes ids are small integers (< 64).
  auto sensorBit = [](const Measurement& m) -> uint64_t {
    return uint64_t{1} << static_cast<unsigned>(m.sensor);
  };

  // Greedy NN: repeatedly take the globally smallest gated pair
  while (true) {
    double best = std::numeric_limits<double>::max();
    size_t bi = 0, bj = 0;
    for (size_t i = 0; i < pool_.size(); ++i) {
      for (size_t j = 0; j < ms.size(); ++j) {
        if (usedMeas[j]) continue;
        if (usedSensors[i] & sensorBit(ms[j])) continue;  // track already has this sensor's measurement
        const Sensor& s = sensors.at(ms[j].sensor);
        const double d = pool_[i].kf.gateDistance(ms[j], s);
        if (d < p_.gate_chi2 && d < best) { best = d; bi = i; bj = j; }
      }
    }
    if (best == std::numeric_limits<double>::max()) break;
    const Sensor& s = sensors.at(ms[bj].sensor);
    pool_[bi].kf.update(ms[bj], s);

    if (usedSensors[bi] == 0) {  // first update this cycle: count the hit once
      pool_[bi].meta.hits++;
      pool_[bi].meta.misses = 0;
      if (pool_[bi].meta.hits >= p_.confirm_hits) pool_[bi].meta.status = TrackStatus::Confirmed;
    }
    usedSensors[bi] |= sensorBit(ms[bj]);
    usedMeas[bj] = 1;
  }

  for (size_t i = 0; i < pool_.size(); ++i)
    if (usedSensors[i] == 0) pool_[i].meta.misses++;

  std::erase_if(pool_, [&](const Internal& tr) {
      return tr.meta.status == TrackStatus::Confirmed
          ? tr.meta.misses >= p_.delete_misses
          : tr.meta.misses >= p_.delete_misses_tentative;
  });

  // Duplicate pruning: two tracks at (nearly) the same place with the same velocity are one target
  std::vector<bool> drop(pool_.size(), false);
  for (size_t i = 0; i < pool_.size(); ++i)
    for (size_t k = i + 1; k < pool_.size(); ++k) {
      if (drop[i] || drop[k]) continue;
      const bool close = (pool_[i].kf.x.head<3>() - pool_[k].kf.x.head<3>()).norm() < 40.0 &&
                         (pool_[i].kf.x.tail<3>() - pool_[k].kf.x.tail<3>()).norm() < 25.0;
      if (!close) continue;
      (pool_[i].meta.hits >= pool_[k].meta.hits ? drop[k] : drop[i]) = true;
    }
  { size_t w = 0; for (size_t i = 0; i < pool_.size(); ++i) if (!drop[i]) { if (w != i) pool_[w] = std::move(pool_[i]); ++w; } pool_.resize(w); }

  // Unassociated measurements spawn tentative tracks
  const size_t pre_existing = pool_.size();
  for (size_t j = 0; j < ms.size(); ++j) {
    if (usedMeas[j]) continue;
    // Several sensors often see the same target on one tick: don't spawn twins
    bool twin = false;
    const Sensor& s = sensors.at(ms[j].sensor);
    for (size_t i = pre_existing; i < pool_.size(); ++i)
      if (pool_[i].kf.gateDistance(ms[j], s) < p_.gate_chi2) { twin = true; break; }
    if (twin) continue;
    Internal n;
    n.meta.id = next_id_++;
    n.meta.tracker_id = t_id;
    n.kf.P.setZero();

    // Initialize the new track's state and covariance
    const Vec3 z_world = s.model().Meas2World(ms[j].pose.position, ms[j].pose.orientation, ms[j].z);
    n.kf.x.head<3>() = z_world;

    Mat3 J = s.model().Meas2WorldJacobian(ms[j].pose.position, ms[j].pose.orientation, ms[j].z);
    Mat3 Rpos = J * ms[j].R * J.transpose();
    n.kf.P.topLeftCorner<3, 3>() = Rpos;
    n.kf.P.bottomRightCorner<3, 3>() = Mat3::Identity() * p_.init_vel_sigma * p_.init_vel_sigma;
    
    n.meta.hits = 1;
    pool_.push_back(n);
  }
}

std::vector<Track> CentralizedTracker::tracks() const {
  std::vector<Track> out;
  for (const auto& tr : pool_) {
    Track t = tr.meta; t.x = tr.kf.x; t.P = tr.kf.P;
    out.push_back(t);
  }
  return out;
}

}  // namespace sfsim
