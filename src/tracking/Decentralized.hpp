#pragma once
#include <deque>
#include <random>
#include <vector>
#include "tracking/IFusion.hpp"
#include <unordered_map>
#include "tracking/CentralizedTracker.hpp"
#include "sim/CommNetwork.hpp"

namespace sfsim {

// Fuse two estimates with unknown cross-correlation. omega picked by minimising trace(P).
void covarianceIntersection(const Vec6& xa, const Mat6& Pa, const Vec6& xb, const Mat6& Pb,
                            Vec6& x, Mat6& P);

struct DecentralizedParams {
  double track_gate_chi2 = 16.81;  // 6 dof, 99%: track-to-track association
  double stale_after_s   = 3.0;    // forget a peer's packet after this long
};

class DecentralizedFusion : public IFusion {
 public:
  DecentralizedFusion(const TrackerParams& tp, size_t n_nodes, CommNetwork& net);
  void step(double t, TrackerId t_id, const std::vector<Sensor>& sensors,
            const std::vector<Measurement>& ms) override;
  std::vector<Track> tracks() const override;

 private:
  struct Node { //TODO: Add the ability to have multiple sensors per node
    SensorId id;
    CentralizedTracker local;
    std::unordered_map<SensorId, Packet> latest;  // newest packet per peer
    std::vector<Track> fused;                     // this node's view of the world
    std::vector<trackPrior> priors;            // neighbours' confirmed tracks waiting for this node's own evidence
    double next_tx = 0;
  };
  std::vector<Track> TrackFusion(const Node& n, double t) const;

  // Refresh priors from the newest packets, drop stale / redundant ones, and create new
  // ones from remote tracks that nothing this node already holds explains.
  void updatePriors(Node& n, double t) const;

  const TrackerParams& tp_;
  CommNetwork& net_;
  std::vector<Node> nodes_;
};

}  // namespace sfsim
