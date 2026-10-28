#pragma once
#include <memory>
#include <random>
#include <vector>
#include "sensors/Sensor.hpp"
#include "core/RateCounter.hpp"
#include "sim/Snapshot.hpp"
#include "tracking/IFusion.hpp"
#include "tracking/TrackerParams.hpp"

namespace sfsim {

struct SimConfig {
  double dt = 0.1;
  uint64_t seed = 1;
};

// Layer C: owns the clock and the execution order. Knows nothing about rendering.
class Simulation {
 public:
  explicit Simulation(SimConfig c) : cfg_(c), rng_(c.seed) {}

  void addTarget(Target t) { targets_.push_back(std::move(t)); }
  void addSensor(SensorConfig c) { sensors_.emplace_back(c); }
  void setFusion(std::unique_ptr<IFusion> f) { fusion_ = std::move(f); }

  void step();
  const Snapshot& snapshot() const { return snap_; }
  double time() const { return t_; }

  // GUI-tweakable at runtime (sliders write here)
  std::vector<Sensor>& sensors() { return sensors_; }
  // The tracker reads these by reference, so edits take effect on the next step.
  TrackerParams& trackerParams() { return *trackerParams_; }

  // Simulation.hpp

  double trackerRateHz() const { return tracker_rate_.hz(); }

  SimConfig cfg_;

 private:
  std::mt19937_64 rng_;
  double t_ = 0;
  std::vector<Target> targets_;
  std::vector<Sensor> sensors_;
  RateCounter tracker_rate_;
  // Heap-allocated so its address survives moving the Simulation (the tracker holds a reference).
  // Declared before fusion_ so it outlives the tracker on destruction.
  std::unique_ptr<TrackerParams> trackerParams_ = std::make_unique<TrackerParams>();
  std::unique_ptr<IFusion> fusion_;
  Snapshot snap_;
};

}  // namespace sfsim