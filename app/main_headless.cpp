#include <cstdio>
#include "sim/Scenario.hpp"
using namespace sfsim;

int main(int argc, char** argv) {
  try {
    const std::string path = (argc > 1) ? argv[1] : std::string(SFSIM_SCENARIO_DIR) + "/default.yaml";
    const ScenarioConfig sc = loadScenario(path);
    Simulation sim = buildSimulation(sc);
    const int steps = static_cast<int>(30.0 / sc.sim.dt);
    for (int k = 0; k < steps; ++k) {
      sim.step();
      if (k % 50 == 49) {
        const auto& s = sim.snapshot();
        int confirmed = 0;
        for (auto& t : s.tracks) confirmed += t.status == TrackStatus::Confirmed;
        std::printf("t=%5.1f truth=%zu meas=%zu tracks=%zu confirmed=%d\n",
                    s.time, s.truth.size(), s.measurements.size(), s.tracks.size(), confirmed);
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}