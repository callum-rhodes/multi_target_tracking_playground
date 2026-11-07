#include "tracking/Decentralised.hpp"

#include <algorithm>

namespace sfsim {

void covarianceIntersection(const Vec6& xa, const Mat6& Pa, const Vec6& xb, const Mat6& Pb,
                            Vec6& x, Mat6& P) {
  const Mat6 Ia = Pa.inverse(), Ib = Pb.inverse();
  double best_tr = 1e300;
  for (int k = 0; k <= 20; ++k) {
    const double w = k / 20.0;
    const Mat6 Pw = (w * Ia + (1 - w) * Ib).inverse();
    const double tr = Pw.trace();
    if (tr < best_tr) {
      best_tr = tr;
      P = Pw;
      x = Pw * (w * Ia * xa + (1 - w) * Ib * xb);
    }
  }
}

namespace { // Private helpers

// Track-to-track Mahalanobis distance. Treats the errors as independent (they aren't
// exactly), which is fine for gating: it only decides *whether* to fuse.
double t2tDist(const Track& a, const Track& b) {
  const Vec6 d = a.x - b.x;
  return d.dot((a.P + b.P).ldlt().solve(d));
}

// Bring an estimate forward by dt with the same constant-velocity model the filters use.
void ageEstimate(Vec6& x, Mat6& P, double dt, double q_accel) {
  EKF f;
  f.x = x;
  f.P = P;
  f.predict(std::max(0.0, dt), q_accel);
  x = f.x;
  P = f.P;
}

// A candidate's estimate, aged to `now`, in Track form so t2tDist can compare it.
Track agedTrack(const trackPrior& p, double now, double q_accel) {
  Track tr;
  tr.x = p.x;
  tr.P = p.P;
  ageEstimate(tr.x, tr.P, now - p.stamp, q_accel);
  return tr;
}

bool explainedBy(const Track& a, const std::vector<Track>& others, double gate) {
  return std::any_of(others.begin(), others.end(),
                     [&](const Track& b) { return t2tDist(a, b) < gate; });
}

trackPrior* findPriors(std::vector<trackPrior>& v, TrackerId from, uint32_t id) {
  for (auto& p : v)
    if (p.source.t_id == from && p.source.id == id) return &p;
  return nullptr;
}

}  // namespace

DecentralisedFusion::DecentralisedFusion(const TrackerParams& tp, size_t n, CommNetwork& net)
    : tp_(tp), net_(net) {
  nodes_.reserve(n);
  for (size_t k = 0; k < n; ++k)
    nodes_.push_back(Node{static_cast<TrackerId>(k), {static_cast<TrackerId>(k)},
                          CentralisedTracker(tp), {}, {}, {}, 0.0});
}

void DecentralisedFusion::step(double t, TrackerId t_id, const std::vector<Sensor>& sensors,
                               const std::vector<Measurement>& ms) {
  // Per-node measurement lists. Each list is fed through the node's phases and shrinks as
  // phases claim measurements, so what is left at the end is exactly what spawns tentatives.
  std::vector<std::vector<Measurement>> byNode(nodes_.size());
  for (const auto& m : ms) byNode.at(m.sensor).push_back(m);

  // 1. Local filtering, first half. Every node is stepped every tick (even with empty input)
  //    so it coasts. The "was there a scan?" decision is made here, from the original input.
  for (size_t k = 0; k < nodes_.size(); ++k) {
    nodes_[k].local.beginScan(t, nodes_[k].id, sensors[k].scannedAt(t));
    byNode[k] = nodes_[k].local.associateConfirmed(sensors, std::move(byNode[k]));
  }

  // 2. Broadcast confirmed local tracks on a slower schedule than the sim tick.
  for (auto& n : nodes_) {
    if (!sensors[n.id].config().enabled) continue;   // a "failed" node goes silent
    if (t + 1e-9 < n.next_tx) continue;
    n.next_tx = t + net_.config().period_s;
    std::vector<Track> mine;
    for (auto& tr : n.local.tracks()) if (tr.status == TrackStatus::Confirmed) mine.push_back(tr);
    for (auto& peer : nodes_) {
      if (peer.id == n.id) continue;
      Packet p; p.from = n.id; p.to = peer.id; p.stamp = t; p.tracks = mine;
      net_.send(t, std::move(p));
    }
  }

  // 3. Receive: keep only the newest packet per peer.
  for (auto& n : nodes_)
    for (auto& p : net_.deliver(t, n.id)) {
      auto& slot = n.latest[p.from];
      if (p.stamp >= slot.stamp) slot = std::move(p);
    }

  // 4. Local filtering, second half, now that this tick's packets are in. The order is the
  //    priority order: priors (informed by a neighbour, tight gate) before existing
  //    tentatives (vague, wide gate), and spawning only from what nobody claimed.
  for (size_t k = 0; k < nodes_.size(); ++k) {
    Node& n = nodes_[k];
    updatePriors(n, t);
    // byNode measurements eaten by each function, so the remainder is passed to the next phase.
    byNode[k] = n.local.associatePriors(sensors, std::move(byNode[k]), n.priors);
    byNode[k] = n.local.associateTentative(sensors, std::move(byNode[k]));
    n.local.spawnTentatives(sensors, byNode[k]);
    n.local.endScan();
  }

  // 5. Rebuild each node's fused view.
  for (auto& n : nodes_) n.fused = TrackFusion(n, t);
}

void DecentralisedFusion::updatePriors(Node& n, double t) const {
  // What this node already tracks itself, after this tick's association against confirmed tracks.
  // (Local tentatives are deliberately left out: if one covers the same target as a candidate,
  // the candidate is tried first, so it takes the measurements and the tentative starves.)
  std::vector<Track> mine;
  for (auto& tr : n.local.tracks()) if (tr.status == TrackStatus::Confirmed) mine.push_back(tr);

  // 1. Refresh the priors we hold from the newest packets. States update; counters do not.
  for (const auto& [from, pkt] : n.latest)
    for (const Track& trIn : pkt.tracks)
      if (trackPrior* p = findPriors(n.priors, from, trIn.id); p && pkt.stamp > p->stamp) {
        p->x = trIn.x;
        p->P = trIn.P;
        p->stamp = pkt.stamp;
      }

  // 2. Forget priors whose source went stale (whatever their hit count), and unpromoted
  //    ones this node now tracks itself. A promoted candidate stays as a marker until stale so
  //    the same neighbour track cannot be re-adopted as a second candidate.
  std::erase_if(n.priors, [&](const trackPrior& p) {
    if (t - p.stamp > tp_.stale_after_s) return true;
    return !p.promoted && explainedBy(agedTrack(p, t, tp_.q_accel), mine, tp_.track_gate_chi2);
  });

  // 3. Create priors from fresh remote tracks that nothing we hold explains: not one of our
  //    own confirmed tracks (track-to-track check), and not a candidate we already created for
  //    the same target from a different neighbour.
  for (const auto& [from, pkt] : n.latest) {
    if (t - pkt.stamp > tp_.stale_after_s) continue;
    for (const Track& trIn : pkt.tracks) {
      if (findPriors(n.priors, from, trIn.id)) continue;   // already known (refreshed above)

      Track aged = trIn;
      ageEstimate(aged.x, aged.P, t - pkt.stamp, tp_.q_accel);
      if (explainedBy(aged, mine, tp_.track_gate_chi2)) continue;

      bool duplicate = false;
      for (const trackPrior& p : n.priors)
        if (t2tDist(aged, agedTrack(p, t, tp_.q_accel)) < tp_.track_gate_chi2) { duplicate = true; break; }
      if (duplicate) continue;

      trackPrior p;
      p.source = {from, trIn.id};
      p.x = trIn.x;
      p.P = trIn.P;
      p.stamp = pkt.stamp;
      n.priors.push_back(p);
    }
  }
}

std::vector<Track> DecentralisedFusion::TrackFusion(const Node& self, double t) const {
  std::vector<Track> fusedTracks;
  for (auto& tr : self.local.tracks()) if (tr.status == TrackStatus::Confirmed) fusedTracks.push_back(tr);

  for (const auto& [from, pkt] : self.latest) {
    if (t - pkt.stamp > tp_.stale_after_s) continue;

    // Bring the peer's tracks forward to "now" (latency + age). Reuse EKF::predict.
    std::vector<Track> remote = pkt.tracks;
    for (auto& trIn : remote) ageEstimate(trIn.x, trIn.P, t - pkt.stamp, tp_.q_accel);

    // Greedy NN association: sort all gated pairs by distance, accept in order.
    struct Cand { double d; size_t i, j; };
    std::vector<Cand> cands;
    for (size_t i = 0; i < fusedTracks.size(); ++i)
      for (size_t j = 0; j < remote.size(); ++j) {
        const double d = t2tDist(fusedTracks[i], remote[j]);
        if (d < tp_.track_gate_chi2) cands.push_back({d, i, j});
      }
    std::sort(cands.begin(), cands.end(), [](auto& a, auto& b) { return a.d < b.d; });

    std::vector<bool> usedV(fusedTracks.size(), false), usedR(remote.size(), false);
    for (auto& c : cands) {
      if (usedV[c.i] || usedR[c.j]) continue;
      usedV[c.i] = usedR[c.j] = true;
      fusedTracks[c.i].fused_from.push_back({from, remote[c.j].id});      // record the incoming belief
      Vec6 x; Mat6 P;
      // --- [Optional] --- Linearise distance at the gate boundary to avoid flickering
      // const double r = std::sqrt(c.d);
      // const double delta = std::sqrt(0.5 * tp_.track_gate_chi2);
      // const double w = (r <= delta) ? 1.0 : delta / r;
      // remote[c.j].P /= w;
      // --- [Optional] ---
      covarianceIntersection(fusedTracks[c.i].x, fusedTracks[c.i].P, remote[c.j].x, remote[c.j].P, x, P);
      fusedTracks[c.i].x = x; fusedTracks[c.i].P = P; fusedTracks[c.i].status = TrackStatus::Fused;
    }
  }
  return fusedTracks;
}

std::vector<Track> DecentralisedFusion::tracks() const {
  std::vector<Track> all;
  for (const auto& n : nodes_) {
    all.insert(all.end(), n.fused.begin(), n.fused.end());   // confirmed: post-fusion track list of this node
    for (const auto& t : n.local.tracks())
      if (t.status == TrackStatus::Unconfirmed) all.push_back(t);  // tentative + unfused: this node's own, unfused
  }
  return all;
}

std::vector<Track> DecentralisedFusion::preFusionTracks() const {
  std::vector<Track> out;
  for (const auto& n : nodes_)
    for (const auto& t : n.local.tracks())
      if (t.status == TrackStatus::Confirmed) out.push_back(t);
  return out;
}

}  // namespace sfsim
