#pragma once
#include <random>
#include <vector>
#include "sensors/Measurement.hpp"
#include "sensors/MeasurementModel.hpp"
#include "core/Target.hpp"

namespace sfsim {

struct SensorConfig {
  SensorId id = 0;
  bool enabled = true;
  MeasType measType = MeasType::Cartesian;
  Pose pose;
  double fov_half_angle = 0.5;   // rad, cone half-angle
  double min_range = 1.0, max_range = 10000.0;
  double update_rate_hz = 5.0;
  Vec3 sigma = Vec3::Constant(5.0);   // per-axis noise: metres for Cartesian, (m, rad, rad) for RangeAzEl
  Vec3 bias = Vec3::Zero();
  double p_detect = 0.9;
  double clutter_rate = 5.0;     // expected false alarms per scan
};

class Sensor {
 public:
  explicit Sensor(SensorConfig cfg)
      : cfg_(std::move(cfg)), model_(cfg_.measType) {}   // model_ must be declared after cfg_

  const SensorConfig& config() const { return cfg_; }
  SensorConfig& config() { return cfg_; }
  // Call after changing config().measType through the mutable reference.
  void syncModel() { if (model_.type() != cfg_.measType) model_ = MeasurementModel(cfg_.measType); }

  const MeasurementModel& model() const { return model_; }

  bool due(double t) const { return t + 1e-9 >= next_time_; }
  void markScanned(double t) { last_scan_t_ = t; next_time_ += 1.0 / cfg_.update_rate_hz; }
  bool scannedAt(double t) const { return std::abs(t - last_scan_t_) < 1e-9; }

  bool inFov(const Vec3& world_pos) const;
  std::vector<Measurement> observe(double t, const std::vector<Target>& targets,
                                   std::mt19937_64& rng) const;

 private:
  SensorConfig cfg_;
  MeasurementModel model_;   // mandatory: no default constructor, so it must be in the init list
  double next_time_ = 0;
  double last_scan_t_ = 0;
};

}  // namespace sfsim
