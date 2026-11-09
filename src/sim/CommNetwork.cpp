#include "sim/CommNetwork.hpp"

namespace sfsim {
  
void CommNetwork::send(double t, Packet p) {
  if (!cfg_.enabled) return;                       // nothing goes on the wire
  std::uniform_real_distribution<double> u(0, 1);
  if (u(rng_) < cfg_.drop_prob) return;
  p.deliver_at = t + cfg_.latency_s;
  in_flight_.push_back(std::move(p));
}

std::vector<Packet> CommNetwork::deliver(double t, SensorId to) {
  if (!cfg_.enabled) {                             // link is down: nothing arrives
    in_flight_.clear();                            // and nothing lingers to arrive later
    return {};
  }
  std::vector<Packet> out;
  std::erase_if(in_flight_, [&](Packet& p) {
    if (t - p.stamp > cfg_.buffer_time_s) return true;  // drop old packets
    // keep: not for this node, or not arrived yet, or connection is lost (will keep retrying)
    if (p.to != to || p.deliver_at > t || !connectionExists(p.from, p.to)) return false;
    out.push_back(std::move(p));                        // take it...
    return true;                                        // ...and remove the moved-from husk
  });
  return out;
}

void CommNetwork::updateTopology(const std::vector<std::pair<TrackerId, Vec3>>& nodePos) {
  double range_sq = cfg_.range_m * cfg_.range_m;

  for (const auto& [updated_id, updated_pos] : nodePos) {
    // 1. Update position
    nodes_[updated_id] = updated_pos;

    // 2. Erase old connections connected to this node
    for (const auto& [other_id, _] : nodes_) {
      if (other_id == updated_id) continue;
      active_connections_.erase({updated_id, other_id});
      active_connections_.erase({other_id, updated_id});
    }

    // 3. Recompute connections for this updated node against all others
    for (const auto& [other_id, other_pos] : nodes_) {
      if (other_id == updated_id) continue;

      // Distance calculation is symmetric, but directed edges can be evaluated
      if ((updated_pos - other_pos).squaredNorm() <= range_sq) {
          active_connections_.insert({updated_id, other_id});
          active_connections_.insert({other_id, updated_id}); // Assuming symmetric range
      }
    }
  }
}

} // namespace sfsim  