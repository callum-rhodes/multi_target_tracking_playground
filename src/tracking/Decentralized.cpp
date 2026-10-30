#include "tracking/Decentralized.hpp"

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

DecentralizedFusion::DecentralizedFusion(const TrackerParams& tp, size_t n, CommNetwork& net)
    : tp_(tp), net_(net) {
  nodes_.reserve(n);
  for (size_t k = 0; k < n; ++k)
    nodes_.push_back(Node{static_cast<SensorId>(k),
                          CentralizedTracker(tp), {}, {}, 0.0});
}

void DecentralizedFusion::step(double t, TrackerId t_id, const std::vector<Sensor>& sensors,
                               const std::vector<Measurement>& ms) {
  // 1. Local filtering: each node sees only its own sensor's measurements.
  //    Every node is stepped every tick (even with empty input) so it coasts.
  std::vector<std::vector<Measurement>> byNode(nodes_.size());
  for (const auto& m : ms) byNode.at(m.sensor).push_back(m);
  for (size_t k = 0; k < nodes_.size(); ++k)
    nodes_[k].local.step(t, nodes_[k].id, sensors, byNode[k]);

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

  // 4. Rebuild each node's fused view.
  for (auto& n : nodes_) n.fused = TrackFusion(n, t);
}

// Track-to-track Mahalanobis distance. Treats the errors as independent (they aren't
// exactly), which is fine for gating: it only decides *whether* to fuse.
static double t2tDist(const Track& a, const Track& b) {
  const Vec6 d = a.x - b.x;
  return d.dot((a.P + b.P).ldlt().solve(d));
}

std::vector<Track> DecentralizedFusion::TrackFusion(const Node& self, double t) const {
  std::vector<Track> fusedTracks;
  for (auto& tr : self.local.tracks()) if (tr.status == TrackStatus::Confirmed) fusedTracks.push_back(tr);

  for (const auto& [from, pkt] : self.latest) {
    if (t - pkt.stamp > tp_.stale_after_s) continue;

    // Bring the peer's tracks forward to "now" (latency + age). Reuse EKF::predict.
    std::vector<Track> remote = pkt.tracks;
    for (auto& r : remote) {
      EKF f; f.x = r.x; f.P = r.P;
      f.predict(t - pkt.stamp, tp_.q_accel);
      r.x = f.x; r.P = f.P;
    }

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
      covarianceIntersection(fusedTracks[c.i].x, fusedTracks[c.i].P, remote[c.j].x, remote[c.j].P, x, P);
      fusedTracks[c.i].x = x; fusedTracks[c.i].P = P; fusedTracks[c.i].status = TrackStatus::Fused;
    }
    // Unmatched remote tracks = targets this node can't see itself. Adopt them.
    // for (size_t j = 0; j < remote.size(); ++j)
    //   if (!usedR[j]) {
    //     remote[j].fused_from.push_back({from, remote[j].id});      // adopted: its only source is the peer's track
    //     remote[j].status = TrackStatus::Confirmed;                 // adopted tracks are always confirmed
    //     remote[j].tracker_id = self.id;                            // restamp AFTER recording the source
    //     fusedTracks.push_back(remote[j]);
    //   }
  }
  return fusedTracks;
}

std::vector<Track> DecentralizedFusion::tracks() const {
  std::vector<Track> all;
  for (const auto& n : nodes_) {
    all.insert(all.end(), n.fused.begin(), n.fused.end());   // confirmed: post-fusion track list of this node
    for (const auto& t : n.local.tracks())
      if (t.status == TrackStatus::Unconfirmed) all.push_back(t);  // tentative + unfused: this node's own, unfused
  }
  return all;
}

}  // namespace sfsim
