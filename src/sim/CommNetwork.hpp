#pragma once
#include <deque>
#include <random>
#include <vector>
#include <unordered_set>

#include "core/Types.hpp" // Vec3, Vec6, Mat6
#include "tracking/IFusion.hpp" // Track, IFusion

namespace sfsim {

struct CommConfig {
  bool enabled = true;       // false = every link is down
  double latency_s = 0.1;
  double drop_prob = 0.05;
  double period_s = 0.5;    // each node broadcasts this often
  double range_m = 10000.0;   // max distance for a link
  double buffer_time_s = 10.0;  // drop packets older than this
  // TODO: bandwidth limit, topology (adjacency matrix)
};

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); 
    }
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
  
  void clearTopology() { nodes_.clear(); active_connections_.clear(); }  // remove all nodes and connections
  void updateTopology(const std::vector<std::pair<TrackerId, Vec3>>& nodePos);  // update the node positions for range checks
  // O(1) Check if directed edge A -> B exists
  bool connectionExists(TrackerId a, TrackerId b) const {
    return active_connections_.find({a, b}) != active_connections_.end();
  }
  const std::unordered_map<TrackerId, Vec3>& nodes() const { return nodes_; }  // node ids and positions
  // void disableConnection(TrackerId from, TrackerId to);  // disable a connection between two nodes
 private:
  CommConfig cfg_;
  std::mt19937_64 rng_;
  std::deque<Packet> in_flight_;
  std::unordered_map<TrackerId, Vec3> nodes_;  // node ids and positions (for range checks)
  std::unordered_set<std::pair<TrackerId, TrackerId>, PairHash> active_connections_;
};

} // namespace sfsim