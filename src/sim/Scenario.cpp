#include "sim/Scenario.hpp"
#include <numbers>
#include <stdexcept>
#include "tracking/CentralizedTracker.hpp"
#include "tracking/Decentralized.hpp"

namespace sfsim {

namespace {
std::unique_ptr<TrajectoryModel> makeTrajectory(const TargetSpec& t) {
  switch (t.trajectory) {
    case TrajectoryType::ConstantVelocity: return std::make_unique<ConstantVelocity>();
    case TrajectoryType::CoordinatedTurn:  return std::make_unique<CoordinatedTurn>(t.turn_rate);
  }
  throw std::invalid_argument("makeTrajectory: unregistered TrajectoryType");
}
}  // namespace

Simulation buildSimulation(const ScenarioConfig& sc) {
  Simulation sim(sc.sim);
  sim.trackerParams() = sc.tracker;     // assign into the sim-owned params: the tracker holds a reference to them
  sim.comms().config() = sc.comms;

  for (std::size_t i = 0; i < sc.sensors.size(); ++i) {
    SensorConfig c = sc.sensors[i];
    // The tracker code indexes `sensors` by id (sensors.at(m.sensor)) and uses ids as bit
    // positions, so id must equal index. Assigning it here makes a mismatch impossible.
    c.id = static_cast<SensorId>(i);
    sim.addSensor(c);
  }

  for (const auto& ts : sc.targets) {
    Target t;
    t.id = ts.id; t.state.pos = ts.pos; t.state.vel = ts.vel;
    t.birth_time = ts.birth_time; t.death_time = ts.death_time;
    t.model = makeTrajectory(ts);
    sim.addTarget(std::move(t));
  }

  // Fusion is created last: it needs the final sensor count and references the sim's params/comms.
  if (sc.decentralised)
    sim.setFusion(std::make_unique<DecentralizedFusion>(sim.trackerParams(), sim.sensors().size(), sim.comms()));
  else
    sim.setFusion(std::make_unique<CentralizedTracker>(sim.trackerParams()));
  return sim;
}
// ==== KEEPING THE OLD BUILT-IN SCENARIO FOR REFERENCE ====
// ScenarioConfig defaultScenario() {
//   using std::numbers::pi;
//   ScenarioConfig sc;
//   sc.name = "Baseline (built-in)";
//   sc.sim = {.dt = 0.02, .seed = 1};
//   sc.decentralised = true;

//   auto sensor = [&](Vec3 pos, double yaw, double rate) {
//     SensorConfig c;
//     c.measType = MeasType::RangeAzEl;
//     c.pose.position = pos;
//     c.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
//     c.fov_half_angle = 0.95; c.max_range = 7500; c.update_rate_hz = rate;
//     c.sigma = Vec3(1, 0.01, 0.01);
//     sc.sensors.push_back(c);
//   };
//   sensor(Vec3(-3000, -2000, 30), 0.6, 5.0);
//   sensor(Vec3( 3000, -2000, 30), pi - 0.6, 5.0);
//   sensor(Vec3(    0,  3500, 30), -pi / 2, 5.0);

//   auto target = [&](TargetId id, Vec3 pos, Vec3 vel, double turn = 0.0, double birth = 0.0, double death = 1e18) {
//     TargetSpec t;
//     t.id = id; t.pos = pos; t.vel = vel; t.birth_time = birth; t.death_time = death;
//     if (turn != 0.0) { t.trajectory = TrajectoryType::CoordinatedTurn; t.turn_rate = turn; }
//     sc.targets.push_back(t);
//   };
//   target(1, Vec3(-1500, -500, 0), Vec3(70, 15, 0));
//   target(2, Vec3( 1500, 1500, 0), Vec3(-60, -10, 0), 0.04);
//   target(3, Vec3(    0,-1000, 0), Vec3(0, 40, 0), -0.03);
//   target(4, Vec3(-2500, 2000, 0), Vec3(60, -20, 0), 0.0, 20.0, 90.0);
//   return sc;
// }

}  // namespace sfsim