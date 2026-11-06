#include "tracking/CentralizedTracker.hpp"

#include <algorithm>
#include <limits>

namespace sfsim {

namespace { // Private helpers

// Bit k set = sensor k. Assumes sensor ids are small integers (< 64); the scenario loader enforces this.
uint64_t sensorBit(const Measurement& m) { return uint64_t{1} << static_cast<unsigned>(m.sensor); }

// Move every measurement that was NOT claimed into a new vector: the "remainder" a phase returns.
std::vector<Measurement> unclaimed(std::vector<Measurement>& ms, const std::vector<char>& used) {
  std::vector<Measurement> ms_remain;
  ms_remain.reserve(ms.size());
  for (size_t j = 0; j < ms.size(); ++j)
    if (!used[j]) ms_remain.push_back(std::move(ms[j]));
  return ms_remain;
}

} // namespace

// ------------------------------------------------------------------ centralised entry point
void CentralizedTracker::step(double t, TrackerId t_id, const std::vector<Sensor>& sensors,
                              const std::vector<Measurement>& ms) {
  bool anyScanned = std::any_of(sensors.begin(), sensors.end(), [&](const Sensor& s) { return s.scannedAt(t); });
  beginScan(t, t_id, anyScanned);
  auto ms_remain = associateConfirmed(sensors, ms);             // copies ms into the by-value parameter
  ms_remain = associateTentative(sensors, std::move(ms_remain));
  spawnTentatives(sensors, ms_remain);
  endScan();
}

// ------------------------------------------------------------------ phases
void CentralizedTracker::beginScan(double t, TrackerId t_id, bool scanned) {
  const double dt = (last_t_ < 0) ? 0.0 : t - last_t_;
  last_t_ = t;
  t_id_ = t_id;
  scanned_ = scanned;
  for (auto& tr : pool_) {
    if (dt > 0) tr.kf.predict(dt, params_.q_accel);
    tr.sensors_used = 0;  // nobody has been updated yet in this scan
  }
}

// Associate confirmed calls associate with the confirmed bool true
std::vector<Measurement> CentralizedTracker::associateConfirmed(const std::vector<Sensor>& sensors,
                                                                std::vector<Measurement> ms) {
  return associate(sensors, std::move(ms), true);
}

// Associate tentative calls associate with the confirmed bool false
std::vector<Measurement> CentralizedTracker::associateTentative(const std::vector<Sensor>& sensors,
                                                                std::vector<Measurement> ms) {
  return associate(sensors, std::move(ms), false);
}

// Shared associate function for confirmed and tentative tracks.
std::vector<Measurement> CentralizedTracker::associate(const std::vector<Sensor>& sensors,
                                                       std::vector<Measurement> ms, bool confirmed) {
  // No scan this tick: coast only. Misses are counted per scan, not per tick.
  if (!scanned_ || ms.empty()) return ms;

  // Uses confirmed bool to determine if we should be associating against confirmed or tentative tracks.
  std::vector<char> eligible(pool_.size(), 0);
  for (size_t i = 0; i < pool_.size(); ++i)
    eligible[i] = (pool_[i].trackInfo.status == TrackStatus::Confirmed) == confirmed;
  std::vector<char> used(ms.size(), 0);

  // Greedy NN: repeatedly take the globally smallest gated pair. Distances are recomputed every
  // round because a track's covariance changes as soon as it is updated.
  while (true) {
    double best = std::numeric_limits<double>::max();
    size_t bi = 0, bj = 0;
    for (size_t i = 0; i < pool_.size(); ++i) {
      if (!eligible[i]) continue;
      for (size_t j = 0; j < ms.size(); ++j) {
        if (used[j]) continue;
        if (pool_[i].sensors_used & sensorBit(ms[j])) continue;  // track already has this sensor's measurement
        const Sensor& s = sensors.at(ms[j].sensor);
        const double d = pool_[i].kf.gateDistance(ms[j], s);
        if (d < params_.gate_chi2 && d < best) { best = d; bi = i; bj = j; }
      }
    }
    if (best == std::numeric_limits<double>::max()) break;

    Internal& tr = pool_[bi];
    tr.kf.update(ms[bj], sensors.at(ms[bj].sensor));
    if (tr.sensors_used == 0) {  // first update this scan: count the hit once
      ++tr.trackInfo.hits;
      tr.trackInfo.misses = 0;
      if (tr.trackInfo.hits >= params_.confirm_hits) tr.trackInfo.status = TrackStatus::Confirmed;
    }
    tr.sensors_used |= sensorBit(ms[bj]);
    used[bj] = 1;
  }
  return unclaimed(ms, used);
}

std::vector<Measurement> CentralizedTracker::associatePriors(const std::vector<Sensor>& sensors,
                                                                 std::vector<Measurement> ms,
                                                                 std::vector<trackPrior>& priors) {
  if (!scanned_) return ms;

  // Age each candidate from the time its estimate was valid to now. Done from the stored
  // estimate every tick (not cumulatively), so there is no covariance drift between packets.
  std::vector<EKF> aged(priors.size());
  for (size_t c = 0; c < priors.size(); ++c) {
    aged[c].x = priors[c].x;
    aged[c].P = priors[c].P;
    aged[c].predict(std::max(0.0, last_t_ - priors[c].stamp), params_.q_accel);
  }

  // Priors are never updated by measurements, so pair distances cannot change once computed.
  // That lets us sort all gated pairs once and accept in order (greedy NN) instead of re-scanning.
  struct Pair { double d; size_t c, m; };
  std::vector<Pair> pairs;
  for (size_t c = 0; c < priors.size(); ++c) {
    if (priors[c].promoted) continue;  // already a real track; only a marker now
    for (size_t m = 0; m < ms.size(); ++m) {
      const double d = aged[c].gateDistance(ms[m], sensors.at(ms[m].sensor));
      if (d < params_.gate_chi2) pairs.push_back({d, c, m});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });

  // A prior claims at most one measurement per scan, and a measurement serves one prior.
  std::vector<char> used(ms.size(), 0), hit(priors.size(), 0);
  for (const Pair& pr : pairs) {
    if (used[pr.m] || hit[pr.c]) continue;
    used[pr.m] = hit[pr.c] = 1;
    trackPrior& c = priors[pr.c];
    ++c.hits;
    c.misses = 0;
    if (c.hits >= params_.prior_confirm_hits)
      promote(c, aged[pr.c], ms[pr.m], sensors.at(ms[pr.m].sensor));
  }

  // Misses: enough in a row wipe the hit counter, but the candidate itself survives. Only the
  // neighbour's track going stale (handled by the fusion layer) removes it.
  for (size_t c = 0; c < priors.size(); ++c) {
    if (priors[c].promoted || hit[c]) continue;  // (promoted this scan => hit[c] is set)
    if (++priors[c].misses >= params_.prior_reset_misses) {
      priors[c].hits = 0;
      priors[c].misses = 0;
    }
  }
  return unclaimed(ms, used);
}

void CentralizedTracker::promote(trackPrior& c, const EKF& aged, const Measurement& m, const Sensor& s) {
  Internal n;
  n.kf = aged;
  // The neighbour's estimate contains none of THIS node's measurements (priors are never
  // broadcast), so a plain Kalman update with our independent measurement is statistically valid.
  n.kf.update(m, s);
  n.trackInfo.id = next_id_++;
  n.trackInfo.tracker_id = t_id_;
  n.trackInfo.hits = c.hits;                      // genuine own-sensor evidence
  n.trackInfo.misses = 0;
  n.trackInfo.status = TrackStatus::Confirmed;    // enters the pool confirmed
  n.sensors_used = sensorBit(m);             // updated this scan, so endScan won't count a miss
  pool_.push_back(std::move(n));
  c.promoted = true;
}

void CentralizedTracker::spawnTentatives(const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms) {
  const size_t pre_existing = pool_.size();
  for (size_t j = 0; j < ms.size(); ++j) {
    // Several sensors often see the same target on one tick: don't spawn twins
    bool twin = false;
    const Sensor& s = sensors.at(ms[j].sensor);
    for (size_t i = pre_existing; i < pool_.size(); ++i)
      if (pool_[i].kf.gateDistance(ms[j], s) < params_.gate_chi2) { twin = true; break; }
    if (twin) continue;

    Internal n;
    n.trackInfo.id = next_id_++;
    n.trackInfo.tracker_id = t_id_;
    n.kf.P.setZero();

    // Initialize the new track's state and covariance
    const Vec3 z_world = s.model().Meas2World(ms[j].pose.position, ms[j].pose.orientation, ms[j].z);
    n.kf.x.head<3>() = z_world;

    const Mat3 J = s.model().Meas2WorldJacobian(ms[j].pose.position, ms[j].pose.orientation, ms[j].z);
    n.kf.P.topLeftCorner<3, 3>() = J * ms[j].R * J.transpose();
    n.kf.P.bottomRightCorner<3, 3>() = Mat3::Identity() * params_.init_vel_sigma * params_.init_vel_sigma;

    n.trackInfo.hits = 1;
    n.sensors_used = sensorBit(ms[j]);  // born from this scan: not a miss
    pool_.push_back(std::move(n));
  }
}

void CentralizedTracker::endScan() {
  if (!scanned_) return;  // coast-only tick: no evidence either way

  // A track that no phase updated this scan has missed.
  for (auto& tr : pool_)
    if (tr.sensors_used == 0) ++tr.trackInfo.misses;

  std::erase_if(pool_, [&](const Internal& tr) {
    return tr.trackInfo.status == TrackStatus::Confirmed
        ? tr.trackInfo.misses >= params_.delete_misses
        : tr.trackInfo.misses >= params_.delete_misses_tentative;
  });

  // Duplicate pruning: two tracks at (nearly) the same place with the same velocity are one target
  std::vector<char> drop(pool_.size(), 0);
  for (size_t i = 0; i < pool_.size(); ++i)
    for (size_t k = i + 1; k < pool_.size(); ++k) {
      if (drop[i] || drop[k]) continue;
      const bool close = (pool_[i].kf.x.head<3>() - pool_[k].kf.x.head<3>()).norm() < 40.0 &&
                         (pool_[i].kf.x.tail<3>() - pool_[k].kf.x.tail<3>()).norm() < 25.0;
      if (!close) continue;
      (pool_[i].trackInfo.hits >= pool_[k].trackInfo.hits ? drop[k] : drop[i]) = 1;
    }
  size_t w = 0;
  for (size_t i = 0; i < pool_.size(); ++i)
    if (!drop[i]) {
      if (w != i) pool_[w] = std::move(pool_[i]);
      ++w;
    }
  pool_.resize(w);
}

std::vector<Track> CentralizedTracker::tracks() const {
  std::vector<Track> out;
  for (const auto& tr : pool_) {
    Track t = tr.trackInfo; t.x = tr.kf.x; t.P = tr.kf.P;
    out.push_back(t);
  }
  return out;
}

}  // namespace sfsim
