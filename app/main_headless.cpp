// Smoke test for the sim core: no GUI. Prints truth vs. track error.
#include <cstdio>
#include "sim/Simulation.hpp"
#include "tracking/CentralizedTracker.hpp"

using namespace sfsim;

int main() {
  Simulation sim({.dt = 0.1, .seed = 42});

  for (int i = 0; i < 2; ++i) {
    SensorConfig sc;
    sc.id = i;
    sc.pose.position = Vec3(i == 0 ? -2000 : 2000, -1000, 0);
    sc.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(i == 0 ? 0.3 : M_PI - 0.3, Vec3::UnitZ()));
    sc.fov_half_angle = 1.0;
    sc.max_range = 6000;
    sc.update_rate_hz = 2.0;
    sim.addSensor(sc);
  }

  Target a; a.id = 1; a.state.pos = Vec3(-500, 0, 1000); a.state.vel = Vec3(60, 10, 0);
  a.model = std::make_unique<ConstantVelocity>();
  Target b; b.id = 2; b.state.pos = Vec3(500, 500, 2000); b.state.vel = Vec3(-50, 0, 0);
  b.model = std::make_unique<CoordinatedTurn>(0.05);
  sim.addTarget(std::move(a));
  sim.addTarget(std::move(b));
  sim.setFusion(std::make_unique<CentralizedTracker>(sim.trackerParams()));

  for (int k = 0; k < 300; ++k) {
    sim.step();
    if (k % 50 == 49) {
      const auto& s = sim.snapshot();
      int confirmed = 0;
      for (auto& t : s.tracks) confirmed += t.status == TrackStatus::Confirmed;
      std::printf("t=%5.1f truth=%zu meas=%zu tracks=%zu confirmed=%d\n",
                  s.time, s.truth.size(), s.measurements.size(), s.tracks.size(), confirmed);
    }
  }
}
