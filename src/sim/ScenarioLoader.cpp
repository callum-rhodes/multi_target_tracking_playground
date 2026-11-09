#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>

#include "sim/Scenario.hpp"

namespace sfsim {
namespace {

constexpr double kDeg2Rad = std::numbers::pi / 180.0;

// ------------------------------------------------------------ small YAML helpers
[[noreturn]] void fail(const YAML::Node& where, const std::string& msg) {
  throw std::runtime_error(msg + " (line " + std::to_string(where.Mark().line + 1) + ")");  // Mark() is 0-based
}

// yaml-cpp silently ignores keys it wasn't asked about, so a typo like "p_detct" would just
// leave the default in place. Rejecting unknown keys turns that into an immediate, clear error.
void checkKeys(const YAML::Node& n, std::initializer_list<const char*> allowed, const std::string& block) {
  if (!n.IsMap()) fail(n, block + " must be a mapping (key: value pairs)");
  for (const auto& kv : n) {
    const std::string key = kv.first.as<std::string>();
    const bool known = std::any_of(allowed.begin(), allowed.end(), [&](const char* a) { return key == a; });
    if (!known) fail(kv.first, "unknown key '" + key + "' in " + block);
  }
}

YAML::Node need(const YAML::Node& n, const std::string& key) {
  const YAML::Node v = n[key];
  if (!v) fail(n, "missing required key '" + key + "'");
  return v;
}

template <typename T>
T req(const YAML::Node& n, const std::string& key) { return need(n, key).as<T>(); }

template <typename T>
T opt(const YAML::Node& n, const std::string& key, T fallback) {
  const YAML::Node v = n[key];
  return v ? v.as<T>() : fallback;
}

double optDeg(const YAML::Node& n, const std::string& key, double fallbackRad) {
  const YAML::Node v = n[key];
  return v ? v.as<double>() * kDeg2Rad : fallbackRad;
}

Vec3 toVec3(const YAML::Node& v, const std::string& key) {
  if (!v.IsSequence() || v.size() != 3) fail(v, "'" + key + "' must be a list of 3 numbers");
  return Vec3(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

Vec3 optVec3(const YAML::Node& n, const std::string& key, const Vec3& fallback) {
  const YAML::Node v = n[key];
  return v ? toVec3(v, key) : fallback;
}

// Parse every element of a YAML list with `parseOne`, prefixing errors with "name[i]: ".
template <typename T, typename F>
void parseList(const YAML::Node& list, const std::string& name, std::vector<T>& out, F parseOne) {
  if (!list.IsSequence()) fail(list, "'" + name + "' must be a list");
  for (std::size_t i = 0; i < list.size(); ++i) {
    try {
      out.push_back(parseOne(list[i]));
    } catch (const std::exception& e) {
      throw std::runtime_error(name + "[" + std::to_string(i) + "]: " + e.what());
    }
  }
}

// ------------------------------------------------------------ blocks
SensorConfig parseSensor(const YAML::Node& n) {
  checkKeys(n, {"enabled", "type", "position", "yaw_deg", "pitch_deg", "roll_deg", "fov_half_angle_deg",
                "min_range", "max_range", "update_rate_hz", "sigma", "bias", "p_detect", "clutter_rate"},
            "sensor");
  SensorConfig c;
  c.enabled = opt(n, "enabled", c.enabled);

  const std::string type = req<std::string>(n, "type");
  if (type == "cartesian") c.measType = MeasType::Cartesian;
  else if (type == "range_az_el") c.measType = MeasType::RangeAzEl;
  else fail(n["type"], "unknown sensor type '" + type + "' (expected cartesian or range_az_el)");

  c.pose.position = toVec3(need(n, "position"), "position");
  c.pose.orientation = Eigen::AngleAxisd(optDeg(n, "yaw_deg", 0.0), Vec3::UnitZ()) *
                       Eigen::AngleAxisd(optDeg(n, "pitch_deg", 0.0), Vec3::UnitY()) *
                       Eigen::AngleAxisd(optDeg(n, "roll_deg", 0.0), Vec3::UnitX());

  c.fov_half_angle = optDeg(n, "fov_half_angle_deg", c.fov_half_angle);
  c.min_range = opt(n, "min_range", c.min_range);
  c.max_range = opt(n, "max_range", c.max_range);
  c.update_rate_hz = opt(n, "update_rate_hz", c.update_rate_hz);
  c.p_detect = opt(n, "p_detect", c.p_detect);
  c.clutter_rate = opt(n, "clutter_rate", c.clutter_rate);

  // sigma is mandatory: SensorConfig's default (5, 5, 5) is fine in metres but would mean
  // 5 *radians* of azimuth noise for a range_az_el sensor.
  c.sigma = toVec3(need(n, "sigma"), "sigma");
  c.bias = optVec3(n, "bias", Vec3::Zero());
  if (c.measType == MeasType::RangeAzEl) {   // file: [m, deg, deg]  ->  sim: (m, rad, rad)
    c.sigma(1) *= kDeg2Rad; c.sigma(2) *= kDeg2Rad;
    c.bias(1) *= kDeg2Rad;  c.bias(2) *= kDeg2Rad;
  }

  if (!(c.sigma.array() > 0.0).all()) fail(n, "sigma entries must be > 0");
  if (c.fov_half_angle <= 0.0 || c.fov_half_angle > std::numbers::pi) fail(n, "fov_half_angle_deg must be in (0, 180]");
  if (c.min_range < 0.0 || c.max_range <= c.min_range) fail(n, "need 0 <= min_range < max_range");
  if (c.update_rate_hz <= 0.0) fail(n, "update_rate_hz must be > 0");
  if (c.p_detect < 0.0 || c.p_detect > 1.0) fail(n, "p_detect must be in [0, 1]");
  if (c.clutter_rate < 0.0) fail(n, "clutter_rate must be >= 0");
  return c;
}

TargetSpec parseTarget(const YAML::Node& n) {
  checkKeys(n, {"id", "position", "velocity", "birth_time", "death_time", "trajectory"}, "target");
  TargetSpec t;
  t.id = req<TargetId>(n, "id");
  t.pos = toVec3(need(n, "position"), "position");
  t.vel = optVec3(n, "velocity", Vec3::Zero());
  t.birth_time = opt(n, "birth_time", t.birth_time);
  t.death_time = opt(n, "death_time", t.death_time);
  if (t.death_time <= t.birth_time) fail(n, "death_time must be after birth_time");

  if (const YAML::Node tr = n["trajectory"]) {
    checkKeys(tr, {"type", "turn_rate_deg_s"}, "trajectory");
    const std::string type = req<std::string>(tr, "type");
    if (type == "constant_velocity") {
      t.trajectory = TrajectoryType::ConstantVelocity;
    } else if (type == "coordinated_turn") {
      t.trajectory = TrajectoryType::CoordinatedTurn;
      t.turn_rate = req<double>(tr, "turn_rate_deg_s") * kDeg2Rad;
    } else {
      fail(tr["type"], "unknown trajectory type '" + type + "'");
    }
  }
  return t;
}

void validate(const ScenarioConfig& sc) {
  if (sc.sim.dt <= 0.0) throw std::runtime_error("sim.dt must be > 0");
  if (sc.sensors.empty()) throw std::runtime_error("at least one sensor is required");
  if (sc.sensors.size() > 64)  // CentralisedTracker tracks "which sensors fused" in a 64-bit mask
    throw std::runtime_error("at most 64 sensors are supported");
  std::set<TargetId> ids;
  for (const auto& t : sc.targets)
    if (!ids.insert(t.id).second) throw std::runtime_error("duplicate target id " + std::to_string(t.id));
  if (sc.comms.drop_prob < 0.0 || sc.comms.drop_prob > 1.0) throw std::runtime_error("comms.drop_prob must be in [0, 1]");
  if (sc.comms.latency_s < 0.0 || sc.comms.period_s <= 0.0) throw std::runtime_error("comms.latency_s must be >= 0 and period_s > 0");
}

ScenarioConfig parse(const YAML::Node& root) {
  checkKeys(root, {"name", "sim", "fusion", "comms", "tracker", "sensors", "targets"}, "scenario");
  ScenarioConfig sc;
  sc.name = opt<std::string>(root, "name", "unnamed");

  if (const YAML::Node s = root["sim"]) {
    checkKeys(s, {"dt", "seed"}, "sim");
    sc.sim.dt = opt(s, "dt", sc.sim.dt);
    sc.sim.seed = opt<uint64_t>(s, "seed", sc.sim.seed);
  }

  const std::string fusion = opt<std::string>(root, "fusion", "centralised");
  if (fusion == "centralised") sc.decentralised = false;
  else if (fusion == "decentralised") sc.decentralised = true;
  else fail(root["fusion"], "fusion must be 'centralised' or 'decentralised'");

  if (const YAML::Node c = root["comms"]) {
    checkKeys(c, {"enabled", "latency_s", "drop_prob", "period_s", "range_m"}, "comms");
    sc.comms.enabled = opt(c, "enabled", sc.comms.enabled);
    sc.comms.latency_s = opt(c, "latency_s", sc.comms.latency_s);
    sc.comms.drop_prob = opt(c, "drop_prob", sc.comms.drop_prob);
    sc.comms.period_s = opt(c, "period_s", sc.comms.period_s);
    sc.comms.range_m = opt(c, "range_m", sc.comms.range_m);
  }

  if (const YAML::Node t = root["tracker"]) {
    checkKeys(t, {"q_accel", "gate_chi2", "confirm_hits", "delete_misses", "delete_misses_tentative",
                  "init_vel_sigma", "stale_after_s", "track_gate_chi2", "prior_confirm_hits", "prior_reset_misses"}, "tracker");
    TrackerParams& p = sc.tracker;
    p.q_accel = opt(t, "q_accel", p.q_accel);
    p.gate_chi2 = opt(t, "gate_chi2", p.gate_chi2);
    p.confirm_hits = opt(t, "confirm_hits", p.confirm_hits);
    p.delete_misses = opt(t, "delete_misses", p.delete_misses);
    p.delete_misses_tentative = opt(t, "delete_misses_tentative", p.delete_misses_tentative);
    p.init_vel_sigma = opt(t, "init_vel_sigma", p.init_vel_sigma);
    p.stale_after_s = opt(t, "stale_after_s", p.stale_after_s);
    p.track_gate_chi2 = opt(t, "track_gate_chi2", p.track_gate_chi2);
    p.prior_confirm_hits = opt(t, "prior_confirm_hits", p.prior_confirm_hits);
    p.prior_reset_misses = opt(t, "prior_reset_misses", p.prior_reset_misses);
  }

  parseList(need(root, "sensors"), "sensors", sc.sensors, parseSensor);
  if (const YAML::Node t = root["targets"]) parseList(t, "targets", sc.targets, parseTarget);  // zero targets is legal
  return sc;
}

}  // namespace

ScenarioConfig loadScenario(const std::string& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path);   // throws on a missing file or bad YAML syntax
    ScenarioConfig sc = parse(root);
    validate(sc);
    return sc;
  } catch (const std::exception& e) {               // YAML::Exception derives from std::runtime_error
    throw std::runtime_error(path + ": " + e.what());
  }
}

}  // namespace sfsim