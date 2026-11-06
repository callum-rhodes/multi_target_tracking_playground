#pragma once
 
namespace sfsim {
 
// Tunable tracker settings. Owned by Simulation (like the sensor configs) and edited live from the GUI;
// trackers hold a reference and read the current values on every step.
struct TrackerParams {
  double q_accel = 80.0;          // process noise intensity
  double gate_chi2 = 11.34;      // chi-square 99% for 3 dof
  int confirm_hits = 10;        // number of hits to confirm a tentative track
  int delete_misses = 10;     // number of misses to delete a confirmed track
  int delete_misses_tentative = 2;  // number of misses to delete a tentative track 
  double init_vel_sigma = 100.0;
  // Decentralised Params
  double stale_after_s = 3.0;    // forget a peer's packet after this long
  double track_gate_chi2 = 16.81;  // 6 dof, 99%: track-to-track association
  // Candidates (a neighbour's confirmed track used as a cue for this node's own sensor)
  int prior_confirm_hits = 5;  // consecutive own-sensor hits before a candidate is promoted to a confirmed track
  int prior_reset_misses = 5;  // this many consecutive misses reset the hit counter (the candidate is NOT deleted)
};
 
}  // namespace sfsim
