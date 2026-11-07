// SDL2 + OpenGL3 + Dear ImGui front end. Reads Simulation::snapshot() only;
// the only writes into the sim are the live sensor-config and tracker-param sliders.
#define GL_SILENCE_DEPRECATION
#define _USE_MATH_DEFINES
#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <filesystem>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include "sim/Simulation.hpp"
#include "sim/Scenario.hpp"
#include "view/Viewport.hpp"
#include "core/maths.hpp"

#ifndef SFSIM_SCENARIO_DIR
#define SFSIM_SCENARIO_DIR "scenarios"   // fallback: relative to the working directory
#endif

using namespace sfsim;

// ---------------------------------------------------------------- scenario
static double sensorYaw(const SensorConfig& c) {
  const Vec3 b = c.pose.orientation * Vec3::UnitX();
  return std::atan2(b.y(), b.x());
}

static const std::string kDefaultScenarioPath = std::string(SFSIM_SCENARIO_DIR) + "/default.yaml";

// Used to initialise App::scenario. Never throws: a broken default file shouldn't stop the app
// from opening, since the user can fix the file and hit Reload.
static ScenarioConfig loadStartupScenario() {
  try {
    return loadScenario(kDefaultScenarioPath);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Could not load default scenario: %s\n", e.what());
    ScenarioConfig empty;
    empty.name = "(default.yaml failed to load, see console)";
    return empty;
  }
}

// ---------------------------------------------------------------- app state
struct Blip { double t; Vec3 p; SensorId sensor; bool clutter; };

struct App {
  ScenarioConfig scenario = loadStartupScenario();
  std::string scenarioPath = kDefaultScenarioPath;   // set even on failure, so Ctrl+R retries after you fix the file
  int seed = (int)scenario.sim.seed;                 // still after `scenario`: initialised in declaration order
  bool decentralised = scenario.decentralised;
  Simulation sim = buildSimulation(scenario);
  Viewport vp;
  bool running = true;
  float speed = 1.0f;
  double acc = 0;

  int followTruth = -1;  // -1 = free view, otherwise follow this target
  int viewingNode = -1;  // -1 = all local, otherwise show only this node's tracks
  static constexpr double kErrHistory = 300.0;  // seconds of error history kept
  static constexpr double kAssocGate  = 1000.0; // m: no confirmed track this close => target counts as untracked
  struct ErrSample { double t; double e; };     // e = NaN while the target is untracked

  float errWindow = 60.0f;
  std::map<uint32_t, std::deque<ErrSample>> errHist;

  bool showGrid = true, showFov = true, showTruth = true, showMeas = true, showTracks = true,
       showTentative = false, showCov = true, showTrails = true, debugClutter = false;
  float measLife = 3.0f;

  std::unordered_map<uint32_t, std::deque<Vec3>> truthTrail;
  std::unordered_map<TrackKey, std::deque<Vec3>> trackTrail;
  std::deque<Blip> blips;

  App() { vp.pan = {0, 500}; vp.zoom = 0.08; }

  void reset() {
    ScenarioConfig sc = scenario;               // the file is the template; the UI's seed / architecture override it
    sc.sim.seed = (uint64_t)seed;
    sc.decentralised = decentralised;
    sim = buildSimulation(sc);
    truthTrail.clear(); trackTrail.clear(); blips.clear(); errHist.clear();
    acc = 0;
  }

  // Returns "" on success, else an error message. On failure the running sim is untouched,
  // because loadScenario() either returns a fully validated config or throws.
  std::string loadFile(const std::string& path) {
    try {
      ScenarioConfig sc = loadScenario(path);
      scenario = std::move(sc);
      scenarioPath = path;
      seed = (int)scenario.sim.seed;
      decentralised = scenario.decentralised;
      followTruth = -1;                         // target / node selections may not exist in the new scenario
      viewingNode = -1;
      reset();
      fitView();
      return {};
    } catch (const std::exception& e) {
      return e.what();
    }
  }

  void stepOnce() {
    sim.step();
    const Snapshot& s = sim.snapshot();
    constexpr size_t kTrail = 400;

    std::unordered_set<uint32_t> liveTruth;
    std::unordered_set<TrackKey> liveTracks;

    for (auto& [id, st] : s.truth) {
      auto& q = truthTrail[id]; q.push_back(st.pos);
      if (q.size() > kTrail) q.pop_front();
      liveTruth.insert(id);
    }
    for (auto& tr : s.tracks) {
      if (tr.status == TrackStatus::Unconfirmed) continue;
      const TrackKey key = trackKey(tr.tracker_id, tr.id);
      auto& q = trackTrail[key]; q.push_back(tr.x.head<3>());
      if (q.size() > kTrail) q.pop_front();
      liveTracks.insert(key);
    }
    std::erase_if(truthTrail, [&](auto& kv) { return !liveTruth.count(kv.first); });
    std::erase_if(trackTrail, [&](auto& kv) { return !liveTracks.count(kv.first); });

    for (auto& m : s.measurements){
      // Get the sensor that produced this measurement, and convert the measurement into world coordinates.
      auto& sensor = sim.sensors()[m.sensor];
      blips.push_back({s.time, sensor.model().Meas2World(m.pose.position, m.pose.orientation, m.z), m.sensor, m.is_clutter});
    } 

    while (!blips.empty() && s.time - blips.front().t > 10.0) blips.pop_front();

    // Position error per target: distance to the nearest confirmed track (NaN if none within the gate)
    for (auto& [id, st] : s.truth) {
      double best = kAssocGate;
      bool found = false;
      for (auto& tr : s.tracks) {
        if (tr.status == TrackStatus::Unconfirmed) continue;
        if (viewingNode >= 0 && tr.tracker_id != (uint32_t)viewingNode) continue;
        const double d = (tr.x.head<3>() - st.pos).norm();
        if (d < best) { best = d; found = true; }
      }
      errHist[id].push_back({s.time, found ? best : std::nan("")});
    }
    for (auto& [id, q] : errHist)
      while (!q.empty() && s.time - q.front().t > kErrHistory) q.pop_front();
    std::erase_if(errHist, [](auto& kv) { return kv.second.empty(); });
  }

  void fitView() {
    Vec3 lo(1e18, 1e18, 0), hi(-1e18, -1e18, 0);
    auto grow = [&](const Vec3& p) {
      lo.x() = std::min(lo.x(), p.x()); lo.y() = std::min(lo.y(), p.y());
      hi.x() = std::max(hi.x(), p.x()); hi.y() = std::max(hi.y(), p.y());
    };
    for (auto& s : sim.sensors()) grow(s.config().pose.position);
    if (sim.snapshot().truth.empty()){ // no targets yet: use the scenario's initial positions
      for (auto& t : scenario.targets) grow(t.pos);
    } else { // targets exist: use the truth positions
      for (auto& [id, st] : sim.snapshot().truth) grow(st.pos);
    }
    if (lo.x() > hi.x()) return;
    vp.pan = {(lo.x() + hi.x()) / 2, (lo.y() + hi.y()) / 2};
    const double ext = std::max({hi.x() - lo.x(), hi.y() - lo.y(), 1000.0}) * 1.3;
    vp.zoom = std::min(vp.screen.x, vp.screen.y) / ext;
  }

  void updateFollow() {
    if (followTruth < 0) return;
    for (const auto& [id, st] : sim.snapshot().truth) {
      if ((int)id == followTruth) {
        vp.pan = {st.pos.x(), st.pos.y()};   // 2D only: z is dropped, like the rest of the view
        return;
      }
    }
    // Not found (not born yet, or dead): leave the camera where it is.
  }
};

// ---------------------------------------------------------------- drawing
static const ImU32 kSensorCol[] = {IM_COL32(90, 170, 255, 255), IM_COL32(255, 120, 200, 255),
                                   IM_COL32(120, 230, 230, 255), IM_COL32(200, 160, 255, 255)};
static ImU32 withAlpha(ImU32 c, int a) { return (c & 0x00FFFFFF) | ((ImU32)a << 24); }

static ImVec2 scr(const Viewport& vp, const Vec3& w) {
  Vec2 s = vp.toScreen(w);
  return ImVec2((float)s.x, (float)s.y);
}

static void drawTrail(ImDrawList* dl, const Viewport& vp, const std::deque<Vec3>& q, ImU32 col) {
  for (size_t i = 1; i < q.size(); ++i) {
    const int a = (int)(30 + 150.0 * i / q.size());
    dl->AddLine(scr(vp, q[i - 1]), scr(vp, q[i]), withAlpha(col, a), 1.2f);
  }
}

static void drawGrid(ImDrawList* dl, const Viewport& vp) {
  const double steps[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000};
  double step = steps[8];
  for (double s : steps) if (s * vp.zoom >= 70) { step = s; break; }
  const Vec3 tl = vp.toWorld({0, 0}), br = vp.toWorld(vp.screen);
  char buf[32];
  for (double x = std::floor(tl.x() / step) * step; x <= br.x(); x += step) {
    const float sx = scr(vp, Vec3(x, 0, 0)).x;
    dl->AddLine(ImVec2(sx, 0), ImVec2(sx, (float)vp.screen.y),
                x == 0 ? IM_COL32(255, 255, 255, 50) : IM_COL32(255, 255, 255, 18));
    std::snprintf(buf, sizeof buf, "%.0f m", x);
    dl->AddText(ImVec2(sx + 3, 3), IM_COL32(255, 255, 255, 90), buf);
  }
  for (double y = std::floor(br.y() / step) * step; y <= tl.y(); y += step) {
    const float sy = scr(vp, Vec3(0, y, 0)).y;
    dl->AddLine(ImVec2(0, sy), ImVec2((float)vp.screen.x, sy),
                y == 0 ? IM_COL32(255, 255, 255, 50) : IM_COL32(255, 255, 255, 18));
    std::snprintf(buf, sizeof buf, "%.0f m", y);
    dl->AddText(ImVec2(3, sy - 14), IM_COL32(255, 255, 255, 90), buf);
  }
}

static void drawCovariance(ImDrawList* dl, const Viewport& vp, const Track& t, ImU32 col) {
  Eigen::Matrix2d C = t.P.topLeftCorner<2, 2>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
  const double a = 2.0 * std::sqrt(std::max(es.eigenvalues()(1), 0.0));
  const double b = 2.0 * std::sqrt(std::max(es.eigenvalues()(0), 0.0));
  const Eigen::Vector2d u = es.eigenvectors().col(1), v = es.eigenvectors().col(0);
  ImVec2 pts[33];
  for (int i = 0; i < 33; ++i) {
    const double th = 2.0 * M_PI * i / 32;
    const Eigen::Vector2d d = u * (a * std::cos(th)) + v * (b * std::sin(th));
    pts[i] = scr(vp, Vec3(t.x.x() + d.x(), t.x.y() + d.y(), 0));
  }
  dl->AddPolyline(pts, 33, col, ImDrawFlags_None, 1.0f);
}

static void drawDashedLine(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col,
                           float thickness = 1.5f, float dash = 6.0f, float gap = 4.0f) {
  const float dx = b.x - a.x, dy = b.y - a.y;
  const float len = std::sqrt(dx * dx + dy * dy);
  if (len < 1e-3f) return;
  if (len > 4000.0f) { dl->AddLine(a, b, col, thickness); return; }  // zoomed far in: don't emit thousands of dashes
  const float ux = dx / len, uy = dy / len;
  for (float s = 0.0f; s < len; s += dash + gap) {
    const float e = std::min(s + dash, len);
    dl->AddLine(ImVec2(a.x + ux * s, a.y + uy * s), ImVec2(a.x + ux * e, a.y + uy * e), col, thickness);
  }
}

// Largest 1/2/5 x 10^k that is <= maxMetres.
static double niceLength(double maxMetres) {
  const double mag = std::pow(10.0, std::floor(std::log10(maxMetres)));
  for (double m : {10.0, 5.0, 2.0, 1.0})
    if (m * mag <= maxMetres) return m * mag;
  return mag;  // not reached: maxMetres >= mag always holds
}

// Map-style scale bar: a horizontal line with a tick at each end and a label above it.
// x_right / y_bar is the bar's right end; it grows leftwards so the label stays put.
static void drawScaleBar(ImDrawList* dl, const Viewport& vp, float x_right, float y_bar) {
  constexpr double kMaxBarPx = 150.0;
  constexpr float kTick = 6.0f;
  const ImU32 col = IM_COL32(255, 255, 255, 200);

  const double metres = niceLength(kMaxBarPx / vp.zoom);
  const float len = (float)(metres * vp.zoom);   // smooth: proportional to zoom between snaps
  const float x0 = x_right - len;

  dl->AddLine(ImVec2(x0, y_bar), ImVec2(x_right, y_bar), col, 1.5f);
  dl->AddLine(ImVec2(x0, y_bar), ImVec2(x0, y_bar - kTick), col, 1.5f);
  dl->AddLine(ImVec2(x_right, y_bar), ImVec2(x_right, y_bar - kTick), col, 1.5f);

  char buf[32];
  if (metres >= 1000.0) std::snprintf(buf, sizeof buf, "%g km", metres / 1000.0);
  else                  std::snprintf(buf, sizeof buf, "%g m", metres);
  const ImVec2 sz = ImGui::CalcTextSize(buf);
  dl->AddText(ImVec2(x_right - sz.x, y_bar - kTick - sz.y - 2.0f), col, buf);
}

static void drawScene(App& app) {
  ImDrawList* dl = ImGui::GetBackgroundDrawList();
  const Viewport& vp = app.vp;
  const Snapshot& snap = app.sim.snapshot();
  char buf[64];

  if (app.showGrid) drawGrid(dl, vp);

  // Sensors + top-down FOV sectors
  for (auto& s : app.sim.sensors()) {
    const SensorConfig& c = s.config();
    if (!c.enabled) continue;
    const ImU32 col = kSensorCol[c.id % 4];
    if (app.showFov) {
      const auto poly = vp.fovSector(c.pose.position, sensorYaw(c), c.fov_half_angle, c.max_range);
      std::vector<ImVec2> pts;
      for (auto& p : poly) pts.push_back(ImVec2((float)p.x, (float)p.y));
      if (c.fov_half_angle <= M_PI / 2)  // convex only; wider cones get an outline
        dl->AddConvexPolyFilled(pts.data(), (int)pts.size(), withAlpha(col, 22));
      dl->AddPolyline(pts.data(), (int)pts.size(), withAlpha(col, 110), ImDrawFlags_Closed, 1.0f);
    }
    const ImVec2 p = scr(vp, c.pose.position);
    dl->AddRectFilled(ImVec2(p.x - 5, p.y - 5), ImVec2(p.x + 5, p.y + 5), col);
    std::snprintf(buf, sizeof buf, "S%u", c.id);
    dl->AddText(ImVec2(p.x + 8, p.y - 7), col, buf);
  }

  // Trails
  if (app.showTrails) {
    if (app.showTruth)
      for (auto& [id, q] : app.truthTrail) drawTrail(dl, vp, q, IM_COL32(80, 220, 120, 255));
    if (app.showTracks)
      for (auto& [id, q] : app.trackTrail) drawTrail(dl, vp, q, IM_COL32(255, 200, 60, 255));
  }

  // Measurements, fading with age
  if (app.showMeas) {
    for (auto& b : app.blips) {
      const double age = snap.time - b.t;
      if (age > app.measLife) continue;
      const int a = (int)(255 * (1.0 - age / app.measLife));
      ImU32 col = kSensorCol[b.sensor % 4];
      if (app.debugClutter && b.clutter) col = IM_COL32(170, 170, 170, 255);
      const ImVec2 p = scr(vp, b.p);
      dl->AddLine(ImVec2(p.x - 3, p.y - 3), ImVec2(p.x + 3, p.y + 3), withAlpha(col, a), 1.5f);
      dl->AddLine(ImVec2(p.x - 3, p.y + 3), ImVec2(p.x + 3, p.y - 3), withAlpha(col, a), 1.5f);
    }
  }

  // Tracks
  if (app.showTracks) {
    for (auto& t : snap.tracks) {
      if (app.viewingNode >= 0 && t.tracker_id != (uint32_t)app.viewingNode) continue;
      if (t.status == TrackStatus::Unconfirmed && !app.showTentative) continue;
      ImU32 col = IM_COL32(150, 150, 150, 200); // default = unconfirmed
      switch (t.status) {
        case TrackStatus::Confirmed: col = IM_COL32(255, 200, 60, 255); break;
        case TrackStatus::Fused: col = IM_COL32(255, 100, 100, 255); break;
      }
      const Vec3 pos = t.x.head<3>(), vel = t.x.tail<3>();
      const ImVec2 p = scr(vp, pos);
      const float r = (float)Viewport::altitudeToRadius(pos.z(), 7.0) + 3.0f;
      dl->AddCircle(p, r, col, 24, 1.5f);
      dl->AddLine(p, scr(vp, pos + Vec3(vel.x(), vel.y(), 0) * 5.0), col, 1.5f);  // 5 s lead
      if (app.showCov && t.status != TrackStatus::Unconfirmed) drawCovariance(dl, vp, t, withAlpha(col, 120));
      std::snprintf(buf, sizeof buf, "#%u_%u", t.tracker_id, t.id);
      dl->AddText(ImVec2(p.x + r + 3, p.y + 2), col, buf);
    }
  }
  // Fusion links (decentralised only)
  // Draw links between fused tracks and their sources.
  if (app.decentralised && app.showTracks) {
    if (app.viewingNode < 0) {   // "all" condition
      std::unordered_map<TrackKey, const Track*> TrackByKey;
      for (const auto& t : snap.tracks)
        if (t.status != TrackStatus::Unconfirmed) TrackByKey[trackKey(t.tracker_id, t.id)] = &t;

      for (const auto& t : snap.tracks) {
        if (t.status != TrackStatus::Fused) continue;
        const ImVec2 a = scr(vp, t.x.head<3>());
        for (const auto& r : t.fused_from) {
          const auto it = TrackByKey.find(trackKey(r.t_id, r.id));
          if (it == TrackByKey.end()) continue;               // the source track isn't in the snapshot
          const ImVec2 b = scr(vp, it->second->x.head<3>());
          // Push the two directions apart a little so A->B and B->A don't overdraw each other
          const float dx = b.x - a.x, dy = b.y - a.y, len = std::max(std::sqrt(dx * dx + dy * dy), 1e-3f);
          const float off = (t.tracker_id < r.t_id) ? 1.5f : -1.5f;
          const ImVec2 n(-dy / len * off, dx / len * off);
          drawDashedLine(dl, ImVec2(a.x + n.x, a.y + n.y), ImVec2(b.x + n.x, b.y + n.y),
                        withAlpha(kSensorCol[t.tracker_id % 4], 200));
        }
      }
    } 
    else {
      const ImU32 col = IM_COL32(90, 200, 255, 255);

      // Fused versions of this node's tracks, keyed so we can link local -> fused.
      std::unordered_map<TrackKey, const Track*> fusedByKey;
      for (const auto& t : snap.tracks)
        if (t.status == TrackStatus::Fused && t.tracker_id == (uint32_t)app.viewingNode)
          fusedByKey[trackKey(t.tracker_id, t.id)] = &t;

      for (const auto& t : snap.unfusedTracks) {
        if (t.tracker_id != (uint32_t)app.viewingNode) continue;
        const ImVec2 p = scr(vp, t.x.head<3>());
        const float r = (float)Viewport::altitudeToRadius(t.x.z(), 7.0) + 3.0f;
        dl->AddRect(ImVec2(p.x - r, p.y - r), ImVec2(p.x + r, p.y + r), col, 0.0f, 0, 1.5f);
        if (app.showCov) drawCovariance(dl, vp, t, withAlpha(col, 120));

        // Dashed line to the fused estimate: its length is how far fusion moved the track.
        if (auto it = fusedByKey.find(trackKey(t.tracker_id, t.id)); it != fusedByKey.end())
          drawDashedLine(dl, p, scr(vp, it->second->x.head<3>()), withAlpha(col, 200));

        std::snprintf(buf, sizeof buf, "L#%u_%u", t.tracker_id, t.id);
        dl->AddText(ImVec2(p.x - r, p.y + r + 2), col, buf);
      }
    }
  }

  // Truth
  if (app.showTruth) {
    const ImU32 col = IM_COL32(80, 220, 120, 255);
    for (auto& [id, st] : snap.truth) {
      const ImVec2 p = scr(vp, st.pos);
      dl->AddCircleFilled(p, (float)Viewport::altitudeToRadius(st.pos.z(), 4.0), col, 16);
      std::snprintf(buf, sizeof buf, "T%u z=%.0f m", id, st.pos.z());
      dl->AddText(ImVec2(p.x + 8, p.y - 16), col, buf);
    }
  }

  // FPS / tracker rate counter, bottom-right corner of the main view
  {
    const ImGuiIO& io = ImGui::GetIO();
    char lines[2][64];
    std::snprintf(lines[0], sizeof lines[0], "tracker %.0f Hz", app.sim.trackerRateHz());
    std::snprintf(lines[1], sizeof lines[1], "%.0f FPS  (%.1f ms)", io.Framerate, 1000.0f / io.Framerate);

    const float line_h = ImGui::GetTextLineHeightWithSpacing();
    const float top = (float)vp.screen.y - 8.0f - line_h * 2.0f;
    float y = top;
    for (auto& s : lines) {
      const ImVec2 sz = ImGui::CalcTextSize(s);
      dl->AddText(ImVec2((float)vp.screen.x - sz.x - 10.0f, y),
                  IM_COL32(255, 255, 255, 140), s);
      y += line_h;
    }

    // Scale bar sits just above the counters, sharing their right margin
    drawScaleBar(dl, vp, (float)vp.screen.x - 10.0f, top - 6.0f);
  }
}

// ---------------------------------------------------------------- scenario browser
struct ScenarioBrowser {
  std::filesystem::path dir = SFSIM_SCENARIO_DIR;
  std::vector<std::filesystem::path> files;
  char pathBuf[512] = {};
  std::string error;

  void refresh() {
    files.clear();
    std::error_code ec;   // non-throwing overloads: a missing folder just yields an empty list
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
      const auto ext = e.path().extension();
      if (e.is_regular_file(ec) && (ext == ".yaml" || ext == ".yml")) files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
  }
};

static void drawScenarioUi(App& app, ScenarioBrowser& br, bool& quit) {
  const ImGuiIO& io = ImGui::GetIO();
  bool wantOpen = false;

  auto reload = [&] {
    br.error = app.loadFile(app.scenarioPath);
    if (!br.error.empty()) wantOpen = true;     // show the error in the dialog
  };

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open scenario...", "Ctrl+O")) wantOpen = true;
      if (ImGui::MenuItem("Reload", "Ctrl+R", false, !app.scenarioPath.empty())) reload();
      ImGui::Separator();
      if (ImGui::MenuItem("Quit")) quit = true;
      ImGui::EndMenu();
    }
    ImGui::TextDisabled("  scenario: %s", app.scenario.name.c_str());
    ImGui::EndMainMenuBar();
  }

  // MenuItem's shortcut text is display-only, so the keys are handled here.
  if (!io.WantTextInput && io.KeyCtrl) {
    if (ImGui::IsKeyPressed(ImGuiKey_O, false)) wantOpen = true;
    if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !app.scenarioPath.empty()) reload();
  }

  // OpenPopup must run in the same ImGui ID scope as BeginPopupModal. Inside the menu bar
  // window it would be in a different scope, so the menu only sets a flag and we open it here.
  if (wantOpen) { br.refresh(); ImGui::OpenPopup("Open scenario"); }

  if (ImGui::BeginPopupModal("Open scenario", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextDisabled("%s", br.dir.string().c_str());
    bool load = false;
    if (ImGui::BeginListBox("##files", ImVec2(480, 150))) {
      for (const auto& f : br.files) {
        const std::string full = f.string();
        if (ImGui::Selectable(f.filename().string().c_str(), full == br.pathBuf,
                              ImGuiSelectableFlags_AllowDoubleClick)) {
          std::snprintf(br.pathBuf, sizeof br.pathBuf, "%s", full.c_str());
          if (ImGui::IsMouseDoubleClicked(0)) load = true;
        }
      }
      ImGui::EndListBox();
    }
    ImGui::SetNextItemWidth(480);
    if (ImGui::InputText("##path", br.pathBuf, sizeof br.pathBuf, ImGuiInputTextFlags_EnterReturnsTrue)) load = true;
    if (ImGui::Button("Open")) load = true;
    ImGui::SameLine();
    const bool cancel = ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape);

    if (load) {
      br.error = app.loadFile(br.pathBuf);
      if (br.error.empty()) ImGui::CloseCurrentPopup();   // on failure stay open so you can fix the file and retry
    }
    if (!br.error.empty()) {
      ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 480.0f);
      ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", br.error.c_str());
      ImGui::PopTextWrapPos();
    }
    if (cancel) { br.error.clear(); ImGui::CloseCurrentPopup(); }
    ImGui::EndPopup();
  }
}

// ---------------------------------------------------------------- UI panels
static void drawControls(App& app) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420, 600), ImGuiCond_FirstUseEver);
  ImGui::Begin("Controls");
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);

  if (ImGui::Button(app.running ? "Pause" : "Resume")) app.running = !app.running;
  ImGui::SameLine();
  if (ImGui::Button("Step")) { app.running = false; app.stepOnce(); }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) app.reset();
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) { app.followTruth = -1; app.fitView(); }
  ImGui::SliderFloat("speed", &app.speed, 0.1f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic);
  ImGui::InputInt("seed (on Reset)", &app.seed);
  ImGui::Text("sim time: %.1f s", app.sim.time());

  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("grid", &app.showGrid); ImGui::SameLine();
    ImGui::Checkbox("sensor FOV", &app.showFov);
    ImGui::Checkbox("truth", &app.showTruth); ImGui::SameLine();
    ImGui::Checkbox("measurements", &app.showMeas);
    ImGui::Checkbox("tracks", &app.showTracks); ImGui::SameLine();
    ImGui::Checkbox("tentative", &app.showTentative);
    ImGui::Checkbox("covariance (2 sigma)", &app.showCov); ImGui::SameLine();
    ImGui::Checkbox("trails", &app.showTrails);
    ImGui::Checkbox("grey out clutter (uses ground truth)", &app.debugClutter);
    ImGui::SliderFloat("measurement fade (s)", &app.measLife, 0.5f, 10.0f);
    float zoomLog = (float)std::log10(app.vp.zoom);
    if (ImGui::SliderFloat("zoom (log px/m)", &zoomLog, -3.0f, 1.0f))
      app.vp.zoom = std::pow(10.0, (double)zoomLog);
    ImGui::SeparatorText("Camera follow");
    ImGui::RadioButton("free", &app.followTruth, -1);
    bool followedAlive = (app.followTruth < 0);
    for (const auto& [id, st] : app.sim.snapshot().truth) {
      ImGui::SameLine();
      char lbl[16];
      std::snprintf(lbl, sizeof lbl, "T%u", id);
      ImGui::RadioButton(lbl, &app.followTruth, (int)id);
      if ((int)id == app.followTruth) followedAlive = true;
    }
    if (!followedAlive)
      ImGui::TextDisabled("T%d is not alive: camera holding position", app.followTruth);
    ImGui::TextDisabled("Wheel = zoom, left/middle drag = pan, Space = pause");
  }

  if (ImGui::CollapsingHeader("Fusion architecture", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("decentralised (nodes exchange tracks)", &app.decentralised)) app.reset();
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Resets the sim with the same seed, so truth and measurements are\n"
                        "identical in both modes: compare the error plot like-for-like.");

    if (app.decentralised) {
      ImGui::SeparatorText("Comms network");
      CommConfig& cc = app.sim.comms().config();
      TrackerParams& tp = app.sim.trackerParams();

      ImGui::Checkbox("comms enabled", &cc.enabled);
      ImGui::BeginDisabled(!cc.enabled);
      float latency = (float) cc.latency_s;
      if (ImGui::SliderFloat("latency (s)", &latency, 0.0f, 2.0f, "%.2f"))
        cc.latency_s = latency;
      float drop = (float)cc.drop_prob;
      if (ImGui::SliderFloat("packet drop prob", &drop, 0.0f, 1.0f, "%.2f"))
        cc.drop_prob = drop;
      float period = (float)cc.period_s;
      if (ImGui::SliderFloat("broadcast period (s)", &period, 0.05f, 5.0f, "%.2f",
                            ImGuiSliderFlags_Logarithmic))
        cc.period_s = period;
      float stale = (float)tp.stale_after_s;
      if (ImGui::SliderFloat("stale after (s)", &stale, 0.5f, 10.0f, "%.1f"))
        tp.stale_after_s = stale;
      ImGui::EndDisabled();

      ImGui::SeparatorText("Track-to-track association");

      float gate = (float)tp.track_gate_chi2;
      if (ImGui::SliderFloat("gate chi2 (6 dof)", &gate, 1.0f, 40.0f, "%.2f"))
        tp.track_gate_chi2 = gate;
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("6 dof: 90%% = 10.64, 95%% = 12.59, 99%% = 16.81, 99.9%% = 22.46");

      ImGui::SeparatorText("View");
      ImGui::RadioButton("all local (unfused)", &app.viewingNode, -1);
      for (size_t k = 0; k < app.sim.sensors().size(); ++k) {
        ImGui::SameLine();
        std::string lbl = "node " + std::to_string(k);
        ImGui::RadioButton(lbl.c_str(), &app.viewingNode, (int)k);
      }
    }
  }

  if (ImGui::CollapsingHeader("Sensors", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (auto& s : app.sim.sensors()) {
      SensorConfig& c = s.config();
      char label[32];
      std::snprintf(label, sizeof label, "Sensor %u", c.id);
      if (!ImGui::TreeNodeEx(label)) continue;
      ImGui::PushID((int)c.id);

      ImGui::Checkbox("enabled", &c.enabled);
      ImGui::BeginDisabled(!c.enabled);   // grey out this sensor's sliders while it's off
      float sg[3] = {(float)c.sigma.x(), (float)c.sigma.y()*f_rad2deg, (float)c.sigma.z()*f_rad2deg};
      bool changed = false;
      changed |= ImGui::SliderFloat("sigma range (m)", &sg[0], 0.1f,   500.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
      changed |= ImGui::SliderFloat("sigma angle (deg)", &sg[1], 0.01f, 60.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
      changed |= ImGui::SliderFloat("sigma elevation (deg)", &sg[2], 0.01f, 60.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
      if (changed) c.sigma = Vec3(sg[0], sg[1]*f_deg2rad, sg[2]*f_deg2rad);
      float pd = (float)c.p_detect;
      if (ImGui::SliderFloat("P(detect)", &pd, 0.0f, 1.0f)) c.p_detect = pd;
      float cl = (float)c.clutter_rate;
      if (ImGui::SliderFloat("clutter / scan", &cl, 0.0f, 20.0f)) c.clutter_rate = cl;
      float rate = (float)c.update_rate_hz;
      if (ImGui::SliderFloat("update rate (Hz)", &rate, 0.2f, 50.0f)) c.update_rate_hz = rate;
      float fov = (float)(c.fov_half_angle * 180.0 / M_PI);
      if (ImGui::SliderFloat("FOV half-angle (deg)", &fov, 5.0f, 180.0f)) c.fov_half_angle = fov * M_PI / 180.0;
      float rng = (float)c.max_range;
      if (ImGui::SliderFloat("max range (m)", &rng, 100.0f, 10000.0f)) c.max_range = rng;
      float yaw = (float)(sensorYaw(c) * 180.0 / M_PI);
      if (ImGui::SliderFloat("heading (deg)", &yaw, -180.0f, 180.0f))
        c.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw * M_PI / 180.0, Vec3::UnitZ()));

      ImGui::EndDisabled();
      ImGui::PopID();
      ImGui::TreePop();
    }
  }

  if (ImGui::CollapsingHeader("Tracker", ImGuiTreeNodeFlags_DefaultOpen)) {
    // Edits go straight into the sim-owned params, which the tracker reads by reference.
    TrackerParams& p = app.sim.trackerParams();
    constexpr ImGuiSliderFlags kLog = ImGuiSliderFlags_Logarithmic;
    auto sliderD = [](const char* label, double* v, double lo, double hi, const char* fmt, ImGuiSliderFlags fl = 0) {
      return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &lo, &hi, fmt, fl);
    };
    sliderD("process noise q_accel", &p.q_accel, 0.1, 1000.0, "%.2f", kLog);
    sliderD("gate chi2 (3 dof)", &p.gate_chi2, 1.0, 40.0, "%.2f");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("3 dof: 95%% = 7.81, 99%% = 11.34, 99.9%% = 16.27");
    ImGui::SliderInt("confirm hits", &p.confirm_hits, 1, 200);
    ImGui::SliderInt("delete misses", &p.delete_misses, 1, 200);
    ImGui::SliderInt("delete misses (tentative)", &p.delete_misses_tentative, 1, 200);
    sliderD("init velocity sigma (m/s)", &p.init_vel_sigma, 1.0, 500.0, "%.1f", kLog);
    ImGui::SliderInt("prior confirm hits (decentralised only)", &p.prior_confirm_hits, 1, 50);
    ImGui::SliderInt("prior reset misses (decentralised only)", &p.prior_reset_misses, 1, 50);
    if (ImGui::Button("Defaults")) p = TrackerParams{};
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

static void drawTelemetry(App& app) {
  const Snapshot& snap = app.sim.snapshot();
  ImGui::SetNextWindowPos(ImVec2(440, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(560, 260), ImGuiCond_FirstUseEver);
  ImGui::Begin("Telemetry");

  int confirmed = 0;
  for (auto& t : snap.tracks) t.status == TrackStatus::Confirmed ? ++confirmed : 0;
  ImGui::Text("truth: %zu   measurements (this tick): %zu   tracks: %d confirmed / %zu total",
              snap.truth.size(), snap.measurements.size(), confirmed, snap.tracks.size());

  if (ImGui::BeginTable("tracks", 9, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    for (const char* h : {"Node ID", "Track ID", "x (m)", "y (m)", "z (m)", "speed", "hits", "sigma pos", "err to truth"})
      ImGui::TableSetupColumn(h);
    ImGui::TableHeadersRow();
    for (auto& t : snap.tracks) {
      if (t.status == TrackStatus::Unconfirmed) continue;
      const Vec3 pos = t.x.head<3>();
      double err = 1e18;  // scoring only: nearest ground-truth distance
      for (auto& [id, st] : snap.truth) err = std::min(err, (st.pos - pos).norm());
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::Text("#%u", t.tracker_id);
      ImGui::TableNextColumn(); ImGui::Text("#%u", t.id);
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.x());
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.y());
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.z());
      ImGui::TableNextColumn(); ImGui::Text("%.1f", t.x.tail<3>().norm());
      ImGui::TableNextColumn(); ImGui::Text("%d", t.hits);
      ImGui::TableNextColumn(); ImGui::Text("%.1f", std::sqrt(t.P.topLeftCorner<3, 3>().trace() / 3.0));
      ImGui::TableNextColumn();
      if (err < 1e17) ImGui::Text("%.1f", err); else ImGui::TextDisabled("-");
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

static void drawErrorPlot(App& app) {
  const ImGuiViewport* vw = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(vw->WorkPos.x + 10, vw->WorkPos.y + vw->WorkSize.y - 230), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(vw->WorkSize.x - 20, 220), ImGuiCond_FirstUseEver);
  ImGui::Begin("Position error");
  ImGui::SetNextItemWidth(200);
  ImGui::SliderFloat("window (s)", &app.errWindow, 10.0f, (float)app.kErrHistory, "%.0f");

  const double t1 = app.sim.snapshot().time, t0 = t1 - app.errWindow;
  double yMax = 10.0;  // auto-scale to the largest visible error, rounded up to 1/2/5 x 10^k
  for (auto& [id, q] : app.errHist)
    for (auto& s : q)
      if (s.t >= t0 && !std::isnan(s.e)) yMax = std::max(yMax, s.e);
  const double mag = std::pow(10.0, std::floor(std::log10(yMax)));
  for (double m : {1.0, 2.0, 5.0, 10.0}) if (yMax <= m * mag) { yMax = m * mag; break; }

  const ImVec2 p0 = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 a(p0.x + 46, p0.y + 18), b(p0.x + avail.x - 8, p0.y + avail.y - 18);
  if (b.x - a.x < 60 || b.y - a.y < 40) { ImGui::End(); return; }
  ImGui::InvisibleButton("##errplot", avail);

  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->PushClipRect(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), true);
  dl->AddRectFilled(a, b, IM_COL32(20, 22, 28, 255));
  char buf[48];
  for (int i = 0; i <= 4; ++i) {  // horizontal grid + y labels
    const float y = b.y - (b.y - a.y) * i / 4.0f;
    dl->AddLine(ImVec2(a.x, y), ImVec2(b.x, y), IM_COL32(255, 255, 255, 35));
    std::snprintf(buf, sizeof buf, "%.0f", yMax * i / 4.0);
    dl->AddText(ImVec2(p0.x + 2, y - 7), IM_COL32(170, 170, 180, 255), buf);
  }
  for (int i = 0; i <= 4; ++i) {  // vertical grid + sim-time labels
    const float x = a.x + (b.x - a.x) * i / 4.0f;
    dl->AddLine(ImVec2(x, a.y), ImVec2(x, b.y), IM_COL32(255, 255, 255, 20));
    std::snprintf(buf, sizeof buf, "%.0f s", t0 + app.errWindow * i / 4.0);
    dl->AddText(ImVec2(x - 14, b.y + 3), IM_COL32(170, 170, 180, 255), buf);
  }
  dl->AddText(ImVec2(p0.x + 2, a.y - 16), IM_COL32(170, 170, 180, 255), "err (m)");

  auto X = [&](double t) { return a.x + (float)((t - t0) / app.errWindow) * (b.x - a.x); };
  auto Y = [&](double e) { return b.y - (float)(std::min(e, yMax) / yMax) * (b.y - a.y); };

  float legendX = a.x + 60;
  std::vector<ImVec2> seg;
  for (auto& [id, q] : app.errHist) {
    const ImU32 col = ImColor::HSV(std::fmod(id * 0.137f, 1.0f), 0.65f, 1.0f);
    auto flush = [&] {
      if (seg.size() >= 2) dl->AddPolyline(seg.data(), (int)seg.size(), col, ImDrawFlags_None, 1.5f);
      else if (seg.size() == 1) dl->AddCircleFilled(seg[0], 2.0f, col);
      seg.clear();
    };
    for (auto& s : q) {
      if (s.t < t0) continue;
      if (std::isnan(s.e)) { flush(); continue; }  // untracked: break the line
      seg.push_back(ImVec2(X(s.t), Y(s.e)));
    }
    flush();

    if (std::isnan(q.back().e)) std::snprintf(buf, sizeof buf, "T%u  untracked", id);
    else std::snprintf(buf, sizeof buf, "T%u  %.1f m", id, q.back().e);
    dl->AddText(ImVec2(legendX, p0.y + 2), col, buf);
    legendX += ImGui::CalcTextSize(buf).x + 16;
  }
  dl->PopClipRect();
  ImGui::End();
}

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
#ifdef __APPLE__
  const char* glsl = "#version 150";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  const char* glsl = "#version 330";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window* window = SDL_CreateWindow("Sensor fusion sim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                        1400, 850, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) { std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError()); return 1; }
  SDL_GLContext gl = SDL_GL_CreateContext(window);
  if (!gl) { std::fprintf(stderr, "GL context failed: %s\n", SDL_GetError()); return 1; }
  SDL_GL_MakeCurrent(window, gl);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init(glsl);

  App app;
  if (argc > 1) {
    const std::string err = app.loadFile(argv[1]);
    if (!err.empty()) std::fprintf(stderr, "%s\n", err.c_str());
  }
  app.fitView();
  bool quit = false;
  ScenarioBrowser browser;

  while (!quit) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      ImGui_ImplSDL2_ProcessEvent(&e);
      if (e.type == SDL_QUIT) quit = true;
      if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE &&
          e.window.windowID == SDL_GetWindowID(window)) quit = true;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    drawScenarioUi(app, browser, quit);            // new; must precede the dockspace
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(),
                                ImGuiDockNodeFlags_PassthruCentralNode);
    app.vp.screen = {io.DisplaySize.x, io.DisplaySize.y};

    // Camera input (only when ImGui isn't using the mouse)
    if (!io.WantCaptureMouse) {
      if (io.MouseWheel != 0.0f)
        app.vp.zoomAbout({io.MousePos.x, io.MousePos.y}, std::pow(1.15, (double)io.MouseWheel));
      if ((io.MouseDown[0] || io.MouseDown[2]) && app.followTruth < 0) {
        app.vp.pan.x -= io.MouseDelta.x / app.vp.zoom;
        app.vp.pan.y += io.MouseDelta.y / app.vp.zoom;
      }
    }
    if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false)) app.running = !app.running;

    // Advance the sim at (real time * speed), capped so a slow frame can't spiral
    if (app.running) {
      app.acc += std::min((double)io.DeltaTime, 0.1) * app.speed;
      int n = 0;
      const double dt = app.sim.cfg_.dt;  // matches SimConfig.dt in makeScenario
      while (app.acc >= dt && n < 100) { app.stepOnce(); app.acc -= dt; ++n; }
      if (n == 100) app.acc = 0;
      app.updateFollow();
    }

    drawScene(app);
    drawControls(app);
    drawTelemetry(app);
    drawErrorPlot(app);

    ImGui::Render();
    glViewport(0, 0, (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x),
               (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y));
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(gl);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}