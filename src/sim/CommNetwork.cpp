#include "sim/CommNetwork.hpp"

namespace sfsim {
  
void CommNetwork::send(double t, Packet p) {
  std::uniform_real_distribution<double> u(0, 1);
  if (u(rng_) < cfg_.drop_prob) return;
  p.deliver_at = t + cfg_.latency_s;
  in_flight_.push_back(std::move(p));
}

std::vector<Packet> CommNetwork::deliver(double t, SensorId to) {
  std::vector<Packet> out;
  std::erase_if(in_flight_, [&](Packet& p) {
    if (p.to != to || p.deliver_at > t) return false;   // keep: not for this node, or not arrived yet
    out.push_back(std::move(p));                        // take it...
    return true;                                        // ...and remove the moved-from husk
  });
  return out;
}

} // namespace sfsim  