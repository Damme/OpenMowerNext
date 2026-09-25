//
// coverage_planner.cpp - OpenMower coverage planner, ROS-agnostic core.
//
// Author: Daniel Wiegert. Same pipeline as the ROS1 coverage_planner node.
// Stages 1-5 are unchanged apart from the node's globals (g_*) moving into
// PlanConfig; ROS I/O (request/response, markers, JSON dump) lives in the caller.
//
// Clearance model: outline/obstacle inputs are base_link traces the mower drove,
// so offset 0 retraces them; outer_offset adds an overgrowth margin. Loop runs
// are clipped at the (slightly deflated) keep-out; fill connectors are validated
// centerline-in-drivable only, no body-footprint check.
//
#include "coverage_planner/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace coverage_planner
{

using namespace geom;

namespace
{

const double kTurnCheckSpacing = 0.05;  // connector validation sample spacing [m]
const double kClipGuard = 0.05;         // keep-out deflation before clipping paths [m]
const double kBlendSlack = 0.01;        // blend validation tolerance outside drivable [m]

// ---- Stage 0: config ---------------------------------------------------------
// Every field is assigned in buildConfig(); the initializers below are safety
// zeros, NOT settings. Configure via Params / the request.
struct PlanConfig {
  double blade_diameter = 0.22;
  double swath_step = 0.22;           // blade * (1 - overlap)
  int n_perimeter_passes = 2;
  double clearance = 0.0;             // inset from the recorded edge; 0 = retrace it
  double obstacle_clearance = 0.0;    // same, from recorded obstacle loops
  double angle_deg = 0.0;
  bool concentric = false;            // request fill type FILL_CONCENTRIC
  std::string edge_side = "right";
  // turning geometry -> turn_footprint(), the max-gap cutoff for U-turns
  double body_width = 0.39, body_length = 0.58, min_turn_radius = 0.0, axle_from_rear = 0.11;
  // node-level settings (ROS1: g_* globals)
  int lane_skip = 2;
  bool optimize_angle = false;
  double path_spacing = 0.1, loop_transition = 6.0;
  double blade_off_x = 0.0, blade_off_y = 0.0;
  double spin_radius() const { return std::hypot(body_length - axle_from_rear, body_width / 2.0); }
  double turn_footprint() const {    // width needed to reverse direction
    return min_turn_radius > 0.0 ? 2.0 * min_turn_radius + body_width : 2.0 * spin_radius();
  }
  double fill_border() const {       // inset where the fill starts (half a swath inside loop n)
    return clearance + (n_perimeter_passes > 0 ? (n_perimeter_passes - 0.5) * swath_step : 0.0);
  }
};

// Clamps as in the ROS1 node's main().
PlanConfig buildConfig(const Request &req, const Params &prm) {
  PlanConfig c;
  const double overlap = std::min(0.9, std::max(0.0, prm.overlap));
  // Params with sentinel defaults follow the request.
  c.blade_diameter = std::max(0.01, prm.tool_width > 0.0 ? prm.tool_width
                                    : req.distance > 1e-6 ? req.distance
                                                          : 0.22);   // neither set: sane default
  c.swath_step = c.blade_diameter * (1.0 - overlap);
  c.n_perimeter_passes = prm.outline_count >= 0 ? prm.outline_count
                                                : std::max(0, req.outline_count);
  c.body_width = prm.body_width; c.body_length = prm.body_length;
  c.min_turn_radius = prm.min_turn_radius; c.axle_from_rear = prm.axle_from_rear;
  c.clearance = prm.clearance >= 0.0 ? prm.clearance
              : req.outer_offset > 1e-6 ? req.outer_offset : 0.0;
  c.obstacle_clearance = prm.obstacle_clearance >= 0.0 ? prm.obstacle_clearance : c.clearance;
  c.angle_deg = req.angle_rad * 180.0 / M_PI;
  c.edge_side = (prm.edge_side == "left") ? "left" : "right";
  if (prm.fill_mode == "concentric")      c.concentric = true;
  else if (prm.fill_mode == "serpentine") c.concentric = false;
  else c.concentric = req.concentric;
  c.lane_skip = std::max(1, prm.lane_skip);
  c.optimize_angle = prm.optimize_sweep_angle;
  c.path_spacing = std::min(0.5, std::max(0.02, prm.path_spacing));
  c.loop_transition = std::max(1.0, prm.loop_transition);
  c.blade_off_x = prm.blade_offset_x; c.blade_off_y = prm.blade_offset_y;
  return c;
}

// ---- planner data --------------------------------------------------------------
struct Pass {
  enum Kind { PERIMETER, OBSTACLE_RING, FILL } kind;
  std::vector<Pt> pts;
  bool is_outline = false;
};

struct Swath { int row; std::vector<Pt> pts; };   // fill lane + field-global scan-row index

struct Field {
  BMultiPolygon outline;                 // pristine boundary
  BMultiPolygon mowable;                 // outline - obstacles (may be multi-lobe)
  BMultiPolygon keepout;                 // obstacles (+) obstacle_clearance, hard no-go
  BMultiPolygon keepout_clip;            // keepout deflated by kClipGuard (see shrinkKeepout)
  BMultiPolygon obstacles_raw;           // union of in-zone recorded obstacle traces
  std::vector<BPolygon> ring_obstacles;  // obstacles that qualify for rings
};

// Every component of `a` covered by `b`. Per-polygon: multipolygon-in-
// multipolygon covered_by is not reliable in Boost.
bool allCoveredBy(const BMultiPolygon &a, const BMultiPolygon &b) {
  if (bg::is_empty(a)) return false;
  for (const auto &p : a) {
    bool c = false; try { c = bg::covered_by(p, b); } catch (...) {}
    if (!c) return false;
  }
  return true;
}

// Deflate each keep-out component by eps so paths that legitimately ride the
// keep-out boundary survive the clip (numeric jitter otherwise shreds them).
// A component too small to shrink stays raw: worst case its rings shred and
// drop as slivers -- never a path across the obstacle.
BMultiPolygon shrinkKeepout(const BMultiPolygon &keepout, double eps) {
  BMultiPolygon out;
  for (const auto &p : keepout) {
    BMultiPolygon s = bufferPolygon(p, -eps);
    if (bg::is_empty(s)) out.push_back(p);
    else for (auto &q : s) out.push_back(std::move(q));
  }
  return out;
}

// DIAGNOSTIC ONLY -- no path surgery. A hairpin-class turn (heading change
// > 100 deg) whose swept body fan reaches into a recorded obstacle: the mower
// may bump there and firmware collision handling takes over. The recorded
// outline is trusted as drivable (it was driven), so only obstacles count.
bool turnHitsObstacle(const Pt &a, const Pt &b, const Pt &c,
                             const BMultiPolygon &obstacles, double r) {
  double h1 = std::atan2(b.y - a.y, b.x - a.x);
  double h2 = std::atan2(c.y - b.y, c.x - b.x);
  double d = std::remainder(h2 - h1, 2.0 * M_PI);
  if (std::fabs(d) < 100.0 * M_PI / 180.0) return false;   // normal driving
  const double beta = 0.42;                                // body corner angular half-extent [rad]
  double from = h1 - (d > 0 ? beta : -beta);
  double span = d + (d > 0 ? 2.0 * beta : -2.0 * beta);
  int steps = std::max(3, (int)std::ceil(std::fabs(span) / 0.26));   // ~15 deg
  for (int i = 0; i <= steps; ++i) {
    double th = from + span * i / steps;
    double cs = std::cos(th), sn = std::sin(th);
    if (multiContains(obstacles, BPoint(b.x + r * cs, b.y + r * sn))) return true;
    if (multiContains(obstacles, BPoint(b.x + 0.5 * r * cs, b.y + 0.5 * r * sn))) return true;
  }
  return false;
}

// Scan the final passes and warn (once, with examples) where hard turns sweep
// into an obstacle. Headings over a ~0.2 m window so a corner split across
// resample vertices still registers.
void warnObstacleTurns(const std::vector<const Pass *> &order, const Field &f,
                              const PlanConfig &cfg, const LogFn &log) {
  if (bg::is_empty(f.obstacles_raw)) return;
  const double r = cfg.spin_radius();
  int hits = 0;
  std::ostringstream where; where.precision(2); where << std::fixed;
  for (const Pass *p : order) {
    std::vector<Pt> pts = resample(p->pts, cfg.path_spacing);
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      const Pt &pa = pts[i >= 2 ? i - 2 : i - 1];
      const Pt &pc = pts[i + 2 < pts.size() ? i + 2 : i + 1];
      if (turnHitsObstacle(pa, pts[i], pc, f.obstacles_raw, r)) {
        if (hits < 5) where << (hits ? ", " : "") << "(" << pts[i].x << "," << pts[i].y << ")";
        ++hits;
        i += 4;                                          // one report per corner
      }
    }
  }
  if (hits && log) {
    std::ostringstream m;
    m << "coverage_planner: " << hits << " hard turn(s) sweep into an obstacle"
      << " (mower may bump; collision handling recovers): " << where.str()
      << (hits > 5 ? ", ..." : "") << ".";
    log(LogLevel::WARN, m.str());
  }
}

// Funnel for every stacked run: clip at the deflated keep-out, drop slivers
// shorter than one cut.
void appendClipped(std::vector<Pass> &out, Pass::Kind kind, bool is_outline,
                          const std::vector<Pt> &run, const Field &f, const PlanConfig &cfg) {
  for (auto &arc : clipPathOutside(run, f.keepout_clip)) {
    if (arc.size() < 2 || pathLength(arc) < cfg.blade_diameter) continue;   // drop slivers
    Pass p; p.kind = kind; p.is_outline = is_outline; p.pts = std::move(arc);
    out.push_back(std::move(p));
  }
}

// Connector/blend validation region: boundary clearance minus keep-outs.
// Crossing mown ground is fine. Empty on failure (fail safe: runs fragment).
BMultiPolygon drivableRegion(const Field &f, const PlanConfig &cfg) {
  BMultiPolygon bound = bufferPolygon(f.outline, -cfg.clearance);
  if (bg::is_empty(f.keepout)) return bound;
  BMultiPolygon d; try { bg::difference(bound, f.keepout, d); } catch (...) { d.clear(); }
  return d;
}

// ---- Stage 1: build field, classify obstacles -----------------------------------
Field buildField(const Request &req, const PlanConfig &cfg) {
  Field f;
  std::vector<Pt> ring;
  for (const auto &p : req.outline) ring.emplace_back(p.x, p.y);
  f.outline = sanitize(makePolygon(ring));
  f.mowable = f.outline;
  if (bg::is_empty(f.outline)) return f;

  for (const auto &hole : req.holes) {
    std::vector<Pt> h;
    for (const auto &p : hole) h.emplace_back(p.x, p.y);
    if (h.size() < 3) continue;
    BMultiPolygon hp = sanitize(makePolygon(h));
    if (bg::is_empty(hp)) continue;

    bool touches = false; try { touches = bg::intersects(hp, f.outline); } catch (...) {}
    if (!touches) continue;                                  // outside the zone
    { BMultiPolygon u; try { bg::union_(f.obstacles_raw, hp, u); f.obstacles_raw = u; } catch (...) {} }

    // Cut from mowable, pre-grown by any excess of obstacle over boundary
    // clearance so the fill border lands at obstacle_clearance from the obstacle.
    double extra = std::max(0.0, cfg.obstacle_clearance - cfg.clearance);
    BMultiPolygon cut = extra > 1e-6 ? bufferPolygon(hp, extra) : hp;
    BMultiPolygon diff; try { bg::difference(f.mowable, cut, diff); } catch (...) {}
    if (!bg::is_empty(diff)) f.mowable = diff;               // ignore a cut that erases everything

    BMultiPolygon grown = bufferPolygon(hp, cfg.obstacle_clearance);   // hard keep-out
    if (!bg::is_empty(grown)) {
      if (bg::is_empty(f.keepout)) f.keepout = grown;
      else { BMultiPolygon u; bg::union_(f.keepout, grown, u); f.keepout = u; }
    }

    // Ring-eligible only if fully inside AND its clearance band clears the
    // boundary -- otherwise the outermost ring would hit the perimeter.
    bool fully_inside = true;
    for (const auto &poly : hp) {
      bool cov = false; try { cov = bg::covered_by(poly, f.outline); } catch (...) {}
      if (!cov) { fully_inside = false; break; }
    }
    if (fully_inside && allCoveredBy(grown, f.outline))
      for (const auto &poly : hp) f.ring_obstacles.push_back(poly);
  }
  f.keepout_clip = shrinkKeepout(f.keepout, kClipGuard);
  return f;
}

// ---- Stages 2+3: boundary loops + obstacle rings, merged per level ------------
// Level k contours are the boundary of R_k = (outline inset b_k) - (ring
// obstacles grown o_k). Far apart that is the classic loop k plus separate ring
// k; when an obstacle sits within a level's band the hole merges into the outer
// contour and level k becomes ONE continuous loop wrapping around the obstacle.
// A ring therefore never crosses its own level's loop line. Level 1 is exempt:
// the closest passes ride the recorded traces untouched (conflicts clip).
// Outer contours emit as PERIMETER, driven deepest-first ending at the
// boundary; hole contours emit as OBSTACLE rings, driven outermost-first ending
// at the obstacle. Winding follows edge_side through wraps automatically.
std::vector<Pass> loopPasses(const Field &f, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(f.outline) || cfg.n_perimeter_passes <= 0) return out;
  const int n = cfg.n_perimeter_passes;
  const bool perim_ccw = (cfg.edge_side == "right");   // CCW -> boundary on the right
  BMultiPolygon lim = bufferPolygon(drivableRegion(f, cfg), kBlendSlack);
  auto blend_ok = [&](const std::vector<Pt> &b) { return pathInside(b, lim, kTurnCheckSpacing); };

  BMultiPolygon obs;                                   // union of ring obstacles
  for (const auto &p : f.ring_obstacles) {
    BMultiPolygon u; try { bg::union_(obs, toMulti(p), u); obs = u; } catch (...) {}
  }

  std::vector<BMultiPolygon> R(n + 1);                 // R[k], k = 1..n
  for (int k = 1; k <= n; ++k) {
    BMultiPolygon base = bufferPolygon(f.outline, -(cfg.clearance + (k - 1) * cfg.swath_step));
    if (k > 1 && !bg::is_empty(obs) && !bg::is_empty(base)) {
      BMultiPolygon g = bufferPolygon(obs, cfg.obstacle_clearance + (k - 1) * cfg.swath_step);
      BMultiPolygon d; try { bg::difference(base, g, d); } catch (...) { d.clear(); }  // fail: level vanishes
      base = d;
    }
    R[k] = base;
  }

  struct Lobe { BPolygon last; std::vector<std::vector<Pt>> rings; };
  auto claim = [](std::vector<Lobe> &lobes, std::vector<char> &claimed, const BPolygon &comp) {
    for (size_t a = 0; a < lobes.size(); ++a) {
      if (claimed[a]) continue;
      bool ov = false; try { ov = bg::intersects(comp, lobes[a].last); } catch (...) {}
      if (ov) { claimed[a] = 1; return (int)a; }
    }
    return -1;
  };

  // Perimeter family: outer contours, levels 1..n (R is monotone shrinking, so
  // an empty level ends it). Runs step inner -> out.
  {
    std::vector<Lobe> lobes;
    auto finish = [&](const Lobe &L) {
      if (L.rings.empty()) return;
      std::vector<std::vector<Pt>> ordered(L.rings.rbegin(), L.rings.rend());  // deepest first
      auto sp = stackLoops(ordered, cfg.path_spacing, cfg.loop_transition, blend_ok);
      if (sp.size() < 2) return;
      appendClipped(out, Pass::PERIMETER, true, sp, f, cfg);
    };
    for (int k = 1; k <= n; ++k) {
      std::vector<BPolygon> comps;
      for (const auto &p : R[k]) if (!bg::is_empty(p)) comps.push_back(p);
      if (comps.empty()) break;                        // deeper levels are emptier
      std::vector<Lobe> next;
      std::vector<char> claimed(lobes.size(), 0);
      for (const auto &comp : comps) {
        int li = claim(lobes, claimed, comp);
        Lobe nl; nl.last = comp;
        std::vector<Pt> r = orientRing(outerRing(comp), perim_ccw);
        if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back(r); }
        else nl.rings = {r};                           // split -> fresh lobe
        next.push_back(std::move(nl));
      }
      for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) finish(lobes[a]);  // lobe ended
      lobes.swap(next);
    }
    for (const auto &L : lobes) finish(L);
  }

  // Obstacle family: hole contours of R_k for k = n..2 (absent while wrapped
  // into the outer contour, appearing as the level drops), plus the untouched
  // level-1 ring on the obstacle traces. Runs step outer -> in.
  if (!bg::is_empty(obs)) {
    std::vector<Lobe> lobes;
    auto finish = [&](const Lobe &L) {
      if (L.rings.empty()) return;
      auto sp = stackLoops(L.rings, cfg.path_spacing, cfg.loop_transition, blend_ok);  // outer -> in
      if (sp.size() < 2) return;
      appendClipped(out, Pass::OBSTACLE_RING, true, sp, f, cfg);
    };
    for (int k = n; k >= 1; --k) {
      std::vector<BPolygon> comps;                     // hole regions as polygons
      if (k == 1) {
        BMultiPolygon g1 = bufferPolygon(obs, cfg.obstacle_clearance);
        for (const auto &p : g1) if (!bg::is_empty(p)) comps.push_back(p);
      } else {
        for (const auto &p : R[k])
          for (auto &hole : innerRings(p)) {
            BPolygon hp = makePolygon(hole);           // corrects winding
            if (!bg::is_empty(hp)) comps.push_back(hp);
          }
      }
      std::vector<Lobe> next;
      std::vector<char> claimed(lobes.size(), 0);
      for (const auto &comp : comps) {
        int li = claim(lobes, claimed, comp);
        Lobe nl; nl.last = comp;
        std::vector<Pt> r = orientRing(outerRing(comp), !perim_ccw);   // obstacle on the edge side
        if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back(r); }
        else nl.rings = {r};                           // appeared / peanut split -> fresh lobe
        next.push_back(std::move(nl));
      }
      for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) finish(lobes[a]);
      lobes.swap(next);
    }
    for (const auto &L : lobes) finish(L);
  }
  return out;
}

// ---- Stage 4: fill -------------------------------------------------------------------

// Parallel swaths over `region` at `angle_deg`, spaced `step`, grouped into
// boustrophedon cells, each swath tagged with a field-global scan-row index.
// A segment continues a cell only on a 1:1 overlap with the previous row;
// split/merge/new opens a fresh cell, and an EMPTY row breaks continuity so
// cells never chain across a gap. ride_bounds (used when there are no perimeter
// loops) pins the first/last row ON the region boundary with even spacing <= step.
std::vector<std::vector<Swath>> decomposeCells(const BMultiPolygon &region,
                                                      double angle_deg, double step,
                                                      bool ride_bounds) {
  std::vector<std::vector<Swath>> cells;
  if (bg::is_empty(region) || step <= 1e-6) return cells;

  Pt cen = centroidOf(region);
  BMultiPolygon rot = rotateMulti(region, -angle_deg, cen);
  double minx, miny, maxx, maxy; boundsOf(rot, minx, miny, maxx, maxy);
  double rad = angle_deg * M_PI / 180.0, cs = std::cos(rad), sn = std::sin(rad);
  auto back = [&](const Pt &p) {                                  // rotated frame -> world
    double dx = p.x - cen.x, dy = p.y - cen.y;
    return Pt(cen.x + dx * cs - dy * sn, cen.y + dx * sn + dy * cs);
  };

  const long cap = 100000;                                        // bad-scale guard
  std::vector<double> ys;
  double span = maxy - miny;
  if (ride_bounds && span > 1e-6) {
    long nr = std::min(cap, (long)std::max(1, (int)std::ceil(span / step - 1e-9)));
    double st = span / nr;                                        // even, <= step
    for (long i = 0; i <= nr; ++i) ys.push_back(miny + i * st);   // rows ON both bounds
  } else {
    for (double y = miny + step / 2.0; y <= maxy + 1e-9 && (long)ys.size() < cap; y += step)
      ys.push_back(y);                                            // centred rows (loops cover the rim)
  }

  struct Seg { double lo, hi; std::vector<Pt> pts; };             // lo/hi = x-extent (rotated)
  std::vector<std::vector<Seg>> stripes;
  for (double y : ys) {
    std::vector<Seg> row;
    for (auto &seg : scanLineClip(rot, y, minx, maxx)) {
      if (seg.size() < 2) continue;
      Pt a = seg.front(), b = seg.back();
      if (a.x > b.x) std::swap(a, b);
      row.push_back({a.x, b.x, {back(a), back(b)}});
    }
    std::sort(row.begin(), row.end(), [](const Seg &A, const Seg &B) { return A.lo < B.lo; });
    stripes.push_back(std::move(row));                            // empty rows too: they break cells
  }

  auto overlap = [&](const Seg &A, const Seg &B) {
    double lo = std::max(A.lo, B.lo), hi = std::min(A.hi, B.hi);
    return (hi - lo) > -0.3 * step;                               // slack bridges tiny gaps
  };
  std::vector<Seg> prev; std::vector<int> prevCell;
  for (size_t ri = 0; ri < stripes.size(); ++ri) {
    std::vector<Seg> &cur = stripes[ri];
    std::vector<int> curCell(cur.size(), -1);
    for (size_t ci = 0; ci < cur.size(); ++ci) {
      int match = -1, nmatch = 0;                            // prev segments this one overlaps
      for (size_t pi = 0; pi < prev.size(); ++pi)
        if (overlap(cur[ci], prev[pi])) { match = (int)pi; ++nmatch; }
      bool cont = false;
      if (nmatch == 1) {                                     // 1:1 both ways -> continue the cell
        int fan = 0; for (auto &c2 : cur) if (overlap(c2, prev[match])) ++fan;
        if (fan == 1) { curCell[ci] = prevCell[match]; cont = true; }
      }
      if (!cont) { curCell[ci] = (int)cells.size(); cells.push_back({}); }  // split/merge/new
      cells[curCell[ci]].push_back({(int)ri, cur[ci].pts});
    }
    prev.swap(cur); prevCell.swap(curCell);
  }
  return cells;
}

// k groups by global row (group o = rows o mod k): adjacent lanes are mowed a
// group apart so cut grass springs back. Tiny cell (<= k lanes) stays one group.
std::vector<std::vector<Swath>> laneSkipGroups(const std::vector<Swath> &sw, int k) {
  std::vector<std::vector<Swath>> groups;
  if (k <= 1 || (int)sw.size() <= k) { groups.push_back(sw); return groups; }
  for (int o = 0; o < k; ++o) {
    std::vector<Swath> g;
    for (const auto &s : sw) if (s.row % k == o) g.push_back(s);
    if (!g.empty()) groups.push_back(std::move(g));
  }
  return groups;
}

// Canonical lane direction: fixed unit vector toward +x (tie-break +y), the
// same for every group and cell. Flip the sign here to run stripes the other way.
Pt laneAxis(const std::vector<Pt> &lane) {
  if (lane.size() < 2) return Pt(1.0, 0.0);
  Pt d(lane.back().x - lane.front().x, lane.back().y - lane.front().y);
  double l = std::hypot(d.x, d.y); if (l < 1e-9) return Pt(1.0, 0.0);
  d.x /= l; d.y /= l;
  if (d.x < -1e-9 || (std::fabs(d.x) < 1e-9 && d.y < 0.0)) { d.x = -d.x; d.y = -d.y; }
  return d;
}

// Chain a cell's swaths into serpentine runs, one per lane_skip group. Drive
// direction = (row/k) parity -> k-wide stripes, phase-aligned field-wide.
// U-turns are fitted to the headland; one that fails even at the tightest
// radius breaks the run (nav repositions).
std::vector<Pass> stitchSwaths(const std::vector<Swath> &sw_in,
                                      const BMultiPolygon &drivable, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (sw_in.empty()) return out;
  const int k = std::max(1, cfg.lane_skip);
  double base_r = cfg.min_turn_radius > 1e-6 ? cfg.min_turn_radius : cfg.swath_step;
  double max_gap = cfg.turn_footprint() * 2.0 + k * cfg.swath_step;
  Pt pref = laneAxis(sw_in[0].pts);                          // shared by all groups/cells
  for (auto &sw : laneSkipGroups(sw_in, k)) {
    for (auto &s : sw) {                                     // orient by the row's k-band
      if (s.pts.size() < 2) continue;
      Pt d(s.pts.back().x - s.pts.front().x, s.pts.back().y - s.pts.front().y);
      double want = ((s.row / k) % 2 == 0) ? 1.0 : -1.0;
      if ((d.x * pref.x + d.y * pref.y) * want < 0.0) std::reverse(s.pts.begin(), s.pts.end());
    }
    Pass run; run.kind = Pass::FILL; run.pts = sw[0].pts;
    for (size_t i = 1; i < sw.size(); ++i) {
      std::vector<Pt> &cur = sw[i].pts;
      const Pt &a = run.pts.back();
      const Pt &ah = run.pts.size() >= 2 ? run.pts[run.pts.size() - 2] : cur.front();
      const Pt &b = cur.front();
      const Pt &bh = cur.size() >= 2 ? cur[1] : cur.back();
      // Widest U-turn whose centerline fits in `drivable`: start gentle, step
      // tighter to the mower's limit.
      double r_top   = std::max(base_r, 0.6 * dist(a, b));
      double r_floor = cfg.min_turn_radius > 1e-6 ? cfg.min_turn_radius : base_r * 0.3;
      std::vector<Pt> conn;
      bool fits = false;
      for (double rr = r_top; ; rr = std::max(r_floor, rr * 0.6)) {
        conn = makeTurn(a, ah, b, bh, rr);
        if (pathInside(conn, drivable, kTurnCheckSpacing)) { fits = true; break; }
        if (rr <= r_floor * 1.0001) break;                   // tightest tried, give up
      }
      if (dist(a, b) < max_gap && fits) {                    // gap gate rejects far-apart lanes
        for (size_t j = 1; j + 1 < conn.size(); ++j) run.pts.push_back(conn[j]);
        for (const auto &p : cur) run.pts.push_back(p);
      } else {
        out.push_back(run); run = Pass(); run.kind = Pass::FILL; run.pts = cur;  // break the run
      }
    }
    out.push_back(run);
  }
  return out;
}

// Concentric fill: nested rings instead of swaths. N = ceil(D / swath_step)
// rings over the max inward depth D, so spacing D/N is even and <= swath_step
// (no gaps); a final half-step ring closes the centre core. lane_skip groups
// rings by depth like the serpentine groups lanes. Winding follows edge_side
// like an obstacle: the uncut core stays on the configured side. Rings trace
// each lobe's OUTER boundary only -- holes are left to the obstacle rings and
// the keep-out clip, so serpentine handles interior obstacles better.
std::vector<Pass> concentricFill(const Field &f, const BMultiPolygon &inner,
                                        const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(inner)) return out;
  double D = inscribedWidth(inner) * 0.5;              // largest inscribed radius
  if (D < 1e-6) return out;
  int N = std::max(1, (int)std::ceil(D / cfg.swath_step - 1e-9));
  double step = D / N;

  std::vector<double> depths;                          // 0, step, ..., then half-step centre core
  for (int i = 0; i < N; ++i) depths.push_back(i * step);
  depths.push_back((N - 0.5) * step);

  // Per-lobe ring stacks tracked as `inner` shrinks and splits (as in
  // loopPasses); depth index keeps lane_skip grouping aligned after splits.
  struct DRing { int depth; std::vector<Pt> ring; };
  struct Lobe { BPolygon last; std::vector<DRing> rings; };
  std::vector<Lobe> lobes, done;
  for (size_t di = 0; di < depths.size(); ++di) {
    BMultiPolygon shrunk = (depths[di] < 1e-9) ? inner : bufferPolygon(inner, -depths[di]);
    std::vector<BPolygon> comps;
    for (const auto &p : shrunk) if (!bg::is_empty(p)) comps.push_back(p);
    std::vector<Lobe> next;
    std::vector<char> claimed(lobes.size(), 0);
    for (const auto &comp : comps) {
      int li = -1;                                     // continue the lobe this ring nests in
      for (size_t a = 0; a < lobes.size(); ++a) {
        if (claimed[a]) continue;
        bool ov = false; try { ov = bg::intersects(comp, lobes[a].last); } catch (...) {}
        if (ov) { li = (int)a; claimed[a] = 1; break; }
      }
      Lobe nl; nl.last = comp;
      if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back({(int)di, outerRing(comp)}); }
      else nl.rings = {{(int)di, outerRing(comp)}};    // split -> fresh lobe
      next.push_back(std::move(nl));
    }
    for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) done.push_back(lobes[a]);
    lobes.swap(next);
  }
  for (auto &L : lobes) done.push_back(L);

  const int k = std::max(1, cfg.lane_skip);
  const bool want_ccw = (cfg.edge_side == "left");     // CW -> uncut core on the right
  BMultiPolygon lim = bufferPolygon(drivableRegion(f, cfg), kBlendSlack);
  auto blend_ok = [&](const std::vector<Pt> &b) { return pathInside(b, lim, kTurnCheckSpacing); };
  for (auto &L : done) {
    for (int o = 0; o < k; ++o) {                      // one atomic run per depth group
      std::vector<std::vector<Pt>> group;
      for (const auto &r : L.rings)
        if (r.depth % k == o) group.push_back(orientRing(r.ring, want_ccw));
      if (group.empty()) continue;
      auto sp = stackLoops(group, cfg.path_spacing, cfg.loop_transition, blend_ok);
      if (sp.size() < 2) continue;
      appendClipped(out, Pass::FILL, false, sp, f, cfg);
    }
  }
  return out;
}

std::vector<Pass> fillArea(const Field &f, const BMultiPolygon &inner, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(inner)) return out;
  if (cfg.concentric) return concentricFill(f, inner, cfg);

  BMultiPolygon drivable = drivableRegion(f, cfg);
  double ang = cfg.optimize_angle ? principalAxisDeg(outerRing(largestComponent(inner)))
                                : cfg.angle_deg;
  double min_run = cfg.blade_diameter;                       // drop slivers shorter than one cut
  bool ride = (cfg.n_perimeter_passes == 0);                 // no loops -> rows ride the boundary
  for (auto &cell : decomposeCells(inner, ang, cfg.swath_step, ride)) {
    if (cell.empty()) continue;
    for (auto &p : stitchSwaths(cell, drivable, cfg)) {
      if (p.pts.size() >= 2 && pathLength(p.pts) >= min_run) out.push_back(std::move(p));
    }
  }
  return out;
}

// ---- Stage 5: order ---------------------------------------------------------------

// Greedy nearest-neighbour, entering each run from its built start only: fill
// runs carry the stripe direction and loops are wound edge-side; never reverse.
void orderByNearest(std::vector<Pass> &passes, Pt start) {
  std::vector<Pass> ordered; ordered.reserve(passes.size());
  std::vector<char> used(passes.size(), 0);
  Pt cur = start;
  for (size_t n = 0; n < passes.size(); ++n) {
    int best = -1; double bestd = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < passes.size(); ++i) {
      if (used[i] || passes[i].pts.empty()) continue;
      double df = dist(cur, passes[i].pts.front());
      if (df < bestd) { bestd = df; best = (int)i; }
    }
    if (best < 0) break;
    used[best] = 1; Pass p = passes[best];
    cur = p.pts.back(); ordered.push_back(std::move(p));
  }
  passes.swap(ordered);
}

// ---- Stage 6: emit ------------------------------------------------------------------
// Resample + blade->base_link shift (ROS1 emitPath without the message building).
std::vector<Pt> emitPoints(const std::vector<Pt> &raw, const PlanConfig &cfg) {
  std::vector<Pt> pts = resample(raw, cfg.path_spacing);
  if (pts.size() < 2) return {};
  return bladePathToBase(pts, cfg.blade_off_x, cfg.blade_off_y);   // no-op if deck centred
}

// ---- pipeline --------------------------------------------------------------------------

Result planImpl(const Request &req, const Params &prm, const LogFn &log) {
  Result res;
  auto info = [&](const std::ostringstream &m) { if (log) log(LogLevel::INFO, m.str()); };
  PlanConfig cfg = buildConfig(req, prm);
  Field f = buildField(req, cfg);
  res.mowable = f.mowable;
  if (bg::is_empty(f.mowable)) {
    if (log) log(LogLevel::ERROR, "coverage_planner: empty mowable area.");
    return res;
  }
  {
    std::ostringstream m;
    m << "coverage_planner: [1/6] field " << bg::area(f.mowable) << " m^2, "
      << f.ring_obstacles.size() << " ring obstacle(s), blade " << cfg.blade_diameter
      << " m (" << (prm.tool_width > 0.0 ? "param" : req.distance > 1e-6 ? "request" : "default")
      << "), overlap " << prm.overlap << " -> swath " << cfg.swath_step
      << " m, clearance " << cfg.clearance
      << (prm.clearance >= 0.0 ? " (param)" : " (request)")
      << " / obstacle " << cfg.obstacle_clearance
      << (prm.obstacle_clearance >= 0.0 ? " (param) m, " : " m, ")
      << cfg.n_perimeter_passes
      << (prm.outline_count >= 0 ? " perimeter pass(es) (param)" : " perimeter pass(es)")
      << ", edge_side=" << cfg.edge_side << ", mow_angle=" << cfg.angle_deg << " deg"
      << (cfg.optimize_angle ? " (overridden by optimize_sweep_angle)" : "") << ".";
    info(m);
  }

  // Fill starts half a swath inside the innermost loop; no loops -> it rides
  // the boundary clearance itself (see ride_bounds). EMPTY inner means the
  // loops converge past the medial axis and cover the zone alone (narrow
  // strip): no fill -- never fill on top of the loops.
  BMultiPolygon inner = bufferPolygon(f.mowable, -cfg.fill_border());

  std::vector<Pass> loops = loopPasses(f, cfg);
  std::vector<Pass> perimeter, obstacles;
  for (auto &p : loops) {
    if (p.kind == Pass::PERIMETER) perimeter.push_back(std::move(p));
    else obstacles.push_back(std::move(p));
  }
  {
    std::ostringstream m;
    m << "coverage_planner: [2/6] perimeter - " << perimeter.size() << " loop-run(s)"
      << (cfg.n_perimeter_passes > 0
          ? ", loop insets " + std::to_string(cfg.clearance) + ".."
            + std::to_string(cfg.clearance + (cfg.n_perimeter_passes - 1) * cfg.swath_step) + " m"
          : std::string()) << ".";
    info(m);
  }
  { std::ostringstream m; m << "coverage_planner: [3/6] obstacles - " << obstacles.size() << " loop-run(s)."; info(m); }
  std::vector<Pass> fill = fillArea(f, inner, cfg);
  {
    std::ostringstream m;
    m << "coverage_planner: [4/6] fill - " << fill.size() << " run(s) ("
      << (cfg.concentric ? "concentric" : "serpentine")
      << (bg::is_empty(inner) ? ", loops cover the zone" : "") << ").";
    info(m);
  }

  // Perimeter first; fill + obstacles ordered together so each obstacle is
  // serviced when the mower is already beside it.
  Pt start(0, 0);
  if (!perimeter.empty() && !perimeter.back().pts.empty()) start = perimeter.back().pts.back();
  std::vector<Pass> work;
  work.insert(work.end(), fill.begin(), fill.end());
  work.insert(work.end(), obstacles.begin(), obstacles.end());
  orderByNearest(work, start);
  { std::ostringstream m; m << "coverage_planner: [5/6] sequenced " << work.size()
                            << " fill+obstacle run(s) by nearest-neighbour."; info(m); }

  std::vector<const Pass *> order;
  for (const auto &p : perimeter) order.push_back(&p);
  for (const auto &p : work) order.push_back(&p);
  warnObstacleTurns(order, f, cfg, log);
  double len_perim = 0, len_obst = 0, len_fill = 0;
  int n_perim = 0, n_obst = 0, n_fill = 0;
  for (const Pass *p : order) {
    std::vector<Pt> pts = emitPoints(p->pts, cfg);
    if (pts.empty()) continue;
    double L = pathLength(p->pts);
    if (p->kind == Pass::PERIMETER)          { len_perim += L; ++n_perim; }
    else if (p->kind == Pass::OBSTACLE_RING) { len_obst  += L; ++n_obst;  }
    else                                     { len_fill  += L; ++n_fill;  }
    PlannedPath out;
    out.kind = p->kind == Pass::PERIMETER ? PlannedPath::PERIMETER
             : p->kind == Pass::OBSTACLE_RING ? PlannedPath::OBSTACLE_RING : PlannedPath::FILL;
    out.is_outline = p->is_outline;
    out.pts = std::move(pts);
    res.paths.push_back(std::move(out));
  }

  if (res.paths.empty() && log)
    log(LogLevel::WARN, "coverage_planner: zone produced no paths (narrower than 2x clearance?).");
  {
    std::ostringstream m;
    m << "coverage_planner: [6/6] emitted " << res.paths.size() << " path(s), total "
      << (len_perim + len_obst + len_fill) << " m   (perimeter " << n_perim << " run/"
      << len_perim << " m, obstacle " << n_obst << " run/" << len_obst << " m, fill "
      << n_fill << " run/" << len_fill << " m).";
    info(m);
  }
  return res;
}

}  // namespace

// Never let an exception reach the caller; an empty result lets it fall back.
Result plan(const Request &req, const Params &params, const LogFn &log) {
  try { return planImpl(req, params, log); }
  catch (const std::exception &e) { if (log) log(LogLevel::ERROR, std::string("coverage_planner threw: ") + e.what()); }
  catch (...) { if (log) log(LogLevel::ERROR, "coverage_planner threw unknown exception."); }
  return Result{};
}

std::vector<double> headings(const std::vector<Pt> &pts) {
  std::vector<double> yaw(pts.size(), 0.0);
  for (size_t i = 0; i + 1 < pts.size(); ++i)
    yaw[i] = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
  if (pts.size() >= 2) yaw.back() = yaw[pts.size() - 2];   // final pose keeps last heading
  return yaw;
}

}  // namespace coverage_planner
