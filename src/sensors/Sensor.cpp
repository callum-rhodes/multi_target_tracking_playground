#include "sensors/Sensor.hpp"
#include "core/maths.hpp"
#include <cmath>

namespace sfsim {

bool Sensor::inFov(const Vec3& p) const {
  const Vec3 los = p - cfg_.pose.position;
  const double r = los.norm();
  if (r < cfg_.min_range || r > cfg_.max_range) return false;
  const Vec3 boresight = cfg_.pose.orientation * Vec3::UnitX();
  return los.dot(boresight) / r >= std::cos(cfg_.fov_half_angle);
}

std::vector<Measurement> Sensor::observe(double t, const std::vector<Target>& targets,
                                         std::mt19937_64& rng) const {
  std::vector<Measurement> out;
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  const Mat3 R = cfg_.sigma.array().square().matrix().asDiagonal();

  for (const auto& tg : targets) {
    if (!tg.alive(t) || !inFov(tg.state.pos)) continue;
    if (u01(rng) > cfg_.p_detect) continue;  // missed detection
    Measurement m;
    m.sensor = cfg_.id; m.type = cfg_.measType; m.time = t; m.R = R; m.truth_id = tg.id;
    m.pose = cfg_.pose;  // sensor pose at measurement time

    const Vec3 z_true = model_.World2Meas(cfg_.pose.position, cfg_.pose.orientation, tg.state.pos);
    m.z = z_true + cfg_.bias +
          Vec3(cfg_.sigma.x() * n01(rng), cfg_.sigma.y() * n01(rng), cfg_.sigma.z() * n01(rng));
    if(m.type == MeasType::RangeAzEl) {m.z(1) = wrapAngle(m.z(1)); m.z(2) = wrapAngle(m.z(2));};

    out.push_back(m);
  }

  // Clutter: uniform in range, uniform direction inside the cone
  if (cfg_.clutter_rate <= 0.0) return out;
  std::poisson_distribution<int> pois(cfg_.clutter_rate);
  const int n_clutter = pois(rng);
  const double cos_fov = std::cos(cfg_.fov_half_angle);
  for (int i = 0; i < n_clutter; ++i) {
    const double ct = cos_fov + (1.0 - cos_fov) * u01(rng);
    const double st = std::sqrt(1.0 - ct * ct);
    const double ph = 2.0 * M_PI * u01(rng);
    const double r = cfg_.min_range + (cfg_.max_range - cfg_.min_range) * u01(rng);
    const Vec3 dir_local(ct, st * std::cos(ph), st * std::sin(ph));
    Measurement m;
    m.sensor = cfg_.id; m.type = cfg_.measType; m.time = t; m.R = R; m.truth_id = 0 ;
    m.is_clutter = true; m.pose = cfg_.pose;  // sensor pose at measurement time
    const Vec3 z_world = cfg_.pose.position + cfg_.pose.orientation * (r * dir_local);
    m.z = model_.World2Meas(cfg_.pose.position, cfg_.pose.orientation, z_world);
    out.push_back(m);
  }
  return out;
}

}  // namespace sfsim
