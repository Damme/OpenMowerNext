#include "mower_logic/footprint.hpp"

#include <algorithm>
#include <cmath>

namespace open_mower_next::mower_logic
{

bool insidePolygon(double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  bool c = false;
  const auto & p = poly.points;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    if ((p[i].y > y) != (p[j].y > y) && x < (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x) c = !c;
  }
  return c;
}

int FootprintCheck::misfit(double x, double y, double yaw, int limit) const
{
  int bad = 0;
  // Centre: inside an operation/navigation area, not in an exclusion.
  bool in_area = false, excluded = false;
  for (const auto & a : map_.areas) {
    if (a.area.polygon.points.size() < 3 || !insidePolygon(x, y, a.area.polygon)) continue;
    if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION) excluded = true;
    in_area = true;
  }
  if (!in_area || excluded) bad += kCentreOutside;
  if (bad > limit) return bad;
  const auto & info = grid_.info;
  if (grid_.data.empty() || info.resolution <= 0.0) return bad;  // no grid yet: centre check only
  auto cell_ok = [&](double px, double py) {
    const int cx = static_cast<int>(std::floor((px - info.origin.position.x) / info.resolution));
    const int cy = static_cast<int>(std::floor((py - info.origin.position.y) / info.resolution));
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height)) return false;
    const int8_t v = grid_.data[static_cast<size_t>(cy) * info.width + static_cast<size_t>(cx)];
    return v >= 0 && v < 100;  // free, rim or blurred edge; not unknown, not lethal
  };
  // The footprint's outline (front corners cut at 45 deg), every 5 cm.
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double f = body_.front, r = -body_.rear, w = body_.half_width;
  const double ch = std::clamp(body_.chamfer, 0.0, std::min(w, f - r));
  const double outline[][2] = {{f, w - ch}, {f - ch, w}, {r, w}, {r, -w}, {f - ch, -w}, {f, -w + ch}};
  constexpr size_t n_pts = sizeof(outline) / sizeof(outline[0]);
  for (size_t i = 0; i < n_pts; ++i) {
    const double u0 = outline[i][0], v0 = outline[i][1];
    const double u1 = outline[(i + 1) % n_pts][0], v1 = outline[(i + 1) % n_pts][1];
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(u1 - u0, v1 - v0) / 0.05)));
    // k < n: the next edge starts with this one's end point.
    for (int k = 0; k < n; ++k) {
      const double u = u0 + (u1 - u0) * k / n, v = v0 + (v1 - v0) * k / n;
      if (!cell_ok(x + u * c - v * s, y + u * s + v * c) && ++bad > limit) return bad;
    }
  }
  return bad;
}

double FootprintCheck::straightRoom(double x, double y, double yaw, int dir, double max, int allowed) const
{
  const double sign = dir > 0 ? 1.0 : -1.0, c = std::cos(yaw), s = std::sin(yaw);
  double ok = 0.0;
  for (double d = 0.05;; d += 0.05) {
    const double dd = std::min(d, max);
    if (misfit(x + sign * dd * c, y + sign * dd * s, yaw, allowed) > allowed) return ok;
    ok = dd;
    if (dd >= max) return ok;
  }
}

std::optional<Move> FootprintCheck::escape(double x, double y, double yaw, double max_drive) const
{
  const int now = misfit(x, y, yaw);
  if (now == 0) return Move{};
  constexpr double kStep = 0.05, kTurnStep = 5.0 * M_PI / 180.0;
  // Shortest straight drive from (px, py, a) that fits; a direction is given up
  // at its first pose worse than now (the way there would pass it).
  auto straight = [&](double px, double py, double a) -> std::optional<double> {
    const double c = std::cos(a), s = std::sin(a);
    bool open[2] = {true, true};
    const double dirs[2] = {-1.0, 1.0};  // back first on a tie
    for (int k = 1; k * kStep <= max_drive + 1e-9 && (open[0] || open[1]); ++k) {
      for (int i = 0; i < 2; ++i) {
        if (!open[i]) continue;
        const double d = dirs[i] * k * kStep;
        const int m = misfit(px + d * c, py + d * s, a, now);
        if (m == 0) return d;
        if (m > now) open[i] = false;
      }
    }
    return std::nullopt;
  };
  if (const auto d = straight(x, y, yaw)) return Move{0.0, *d};
  // Turns in place, both ways in 5 deg steps up to 180; a way is given up at
  // its first heading worse than now.
  std::optional<Move> turn_and_drive;
  bool open[2] = {true, true};
  const double signs[2] = {1.0, -1.0};
  for (int k = 1; k <= 36 && (open[0] || open[1]); ++k) {
    for (int i = 0; i < 2; ++i) {
      if (!open[i]) continue;
      const double t = signs[i] * k * kTurnStep;
      const int m = misfit(x, y, yaw + t, now);
      if (m == 0) return Move{t, 0.0};
      if (m > now) {
        open[i] = false;
        continue;
      }
      if (!turn_and_drive) {
        if (const auto d = straight(x, y, yaw + t)) turn_and_drive = Move{t, *d};
      }
    }
  }
  return turn_and_drive;
}

}  // namespace open_mower_next::mower_logic
