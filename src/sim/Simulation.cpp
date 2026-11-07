#include "sim/Simulation.hpp"

namespace sfsim {

void Simulation::step() {
  t_ += cfg_.dt;

  // 1. true physics
  for (auto& tg : targets_) if (tg.alive(t_)) tg.model->advance(tg.state, cfg_.dt);

  // 2+3. sensors observe truth and corrupt it
  std::vector<Measurement> all;
  for (auto& s : sensors_) {
    if (!s.due(t_)) continue;
    if (!s.config().enabled) continue;
    auto m = s.observe(t_, targets_, rng_);
    all.insert(all.end(), m.begin(), m.end());
    s.markScanned(t_);
  }

  // 4. fusion (centralised or decentralised, behind IFusion)
  if (fusion_) fusion_->step(t_, 0, sensors_, all);

  // 5. publish a snapshot for the view
  snap_.time = t_;
  snap_.truth.clear();
  for (const auto& tg : targets_) if (tg.alive(t_)) snap_.truth.emplace_back(tg.id, tg.state);
  snap_.measurements = std::move(all);
  snap_.tracks = fusion_ ? fusion_->tracks() : std::vector<Track>{};
  snap_.unfusedTracks = fusion_ ? fusion_->preFusionTracks() : std::vector<Track>{};

  tracker_rate_.tick();
}

}  // namespace sfsim
