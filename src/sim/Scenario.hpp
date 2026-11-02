#pragma once
#include <string>
#include <vector>
#include "core/Target.hpp"
#include "sensors/Sensor.hpp"
#include "sim/CommNetwork.hpp"
#include "sim/Simulation.hpp"
#include "tracking/TrackerParams.hpp"

namespace sfsim {

enum class TrajectoryType { ConstantVelocity, CoordinatedTurn };

// Copyable description of a target. Target itself owns a unique_ptr, so it can't be copied.
struct TargetSpec {
  TargetId id = 0;
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  double birth_time = 0, death_time = 1e18;
  TrajectoryType trajectory = TrajectoryType::ConstantVelocity;
  double turn_rate = 0;  // rad/s, CoordinatedTurn only
};

// Everything needed to (re)create a simulation. Plain data: angles are in radians here;
// the file format uses degrees and the loader converts.
struct ScenarioConfig {
  std::string name = "unnamed";
  SimConfig sim;
  bool decentralised = false;
  CommConfig comms;
  TrackerParams tracker;
  std::vector<SensorConfig> sensors;   // sensor id == index in this vector (see buildSimulation)
  std::vector<TargetSpec> targets;
};

ScenarioConfig loadScenario(const std::string& path);   // throws std::runtime_error
Simulation buildSimulation(const ScenarioConfig& sc);

}  // namespace sfsim