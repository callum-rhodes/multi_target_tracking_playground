#pragma once
#include <deque>
#include <random>
#include <vector>

#include "core/Types.hpp" // Vec3, Vec6, Mat6
#include "tracking/IFusion.hpp" // Track, IFusion

namespace sfsim {

struct CommConfig {
  bool enabled = true;       // false = every link is down
  double latency_s = 0.1;
  double drop_prob = 0.05;
  double period_s = 0.5;    // each node broadcasts this often
  double range_m = 10000.0;   // max distance for a link
  // TODO: bandwidth limit, topology (adjacency matrix)
};

struct Packet {
  SensorId from = 0, to = 0;
  double stamp = 0;        // time the tracks are valid at
  double deliver_at = 0;
  std::vector<Track> tracks;
};

// Simulated comms graph: send() now, deliver() returns what has arrived by t.
class CommNetwork {
 public:
  CommNetwork(CommConfig c, uint64_t seed) : cfg_(c), rng_(seed) {}
  void send(double t, Packet p);
  std::vector<Packet> deliver(double t, SensorId to);
  CommConfig& config() { return cfg_; }
 private:
  CommConfig cfg_;
  std::mt19937_64 rng_;
  std::deque<Packet> in_flight_;
};

} // namespace sfsim