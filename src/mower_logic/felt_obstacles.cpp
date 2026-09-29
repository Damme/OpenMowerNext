#include "mower_logic/felt_obstacles.hpp"

#include <algorithm>
#include <cmath>

namespace open_mower_next::mower_logic
{

namespace
{
// Front outline in base_link (left to right), each segment with its outward normal.
struct Segment
{
  Point a, b, n;
};

std::vector<Segment> frontOutline(const Body & body, int side)
{
  const double f = body.front, w = body.half_width;
  const double ch = std::clamp(body.chamfer, 0.0, std::min(w, f));
  const double side_len = 0.05;  // of the long sides, behind the cut corners
  const double k = std::sqrt(0.5);
  std::vector<Segment> all = {
    {{f - ch - side_len, w}, {f - ch, w}, {0.0, 1.0}},         // left side
    {{f - ch, w}, {f, w - ch}, {k, k}},                        // front-left corner
    {{f, w - ch}, {f, 0.0}, {1.0, 0.0}},                       // front face, left half
    {{f, 0.0}, {f, -(w - ch)}, {1.0, 0.0}},                    // front face, right half
    {{f, -(w - ch)}, {f - ch, -w}, {k, -k}},                   // front-right corner
    {{f - ch, -w}, {f - ch - side_len, -w}, {0.0, -1.0}},      // right side
  };
  // Arcing into the obstacle, the leading corner touches: that side, its cut
  // corner and the outer half of the front face on that side.
  if (side > 0) {
    all.resize(3);
    all[2].b = {f, (w - ch) / 2.0};
  }
  if (side < 0) {
    all.erase(all.begin(), all.begin() + 3);
    all[0].a = {f, -(w - ch) / 2.0};
  }
  return all;
}

Point toMap(const Contact & c, double u, double v)
{
  const double cs = std::cos(c.yaw), sn = std::sin(c.yaw);
  return {c.x + u * cs - v * sn, c.y + u * sn + v * cs};
}
}  // namespace

std::vector<Point> contactMarks(const Contact & c, const MarkShape & s)
{
  std::vector<Point> out;
  const auto segs = frontOutline(s.body, c.side);
  const int layers = std::max(1, static_cast<int>(std::round(s.depth / s.step)) + 1);
  for (size_t si = 0; si < segs.size(); ++si) {
    const auto & g = segs[si];
    const double len = std::hypot(g.b.first - g.a.first, g.b.second - g.a.second);
    const int n = std::max(1, static_cast<int>(std::ceil(len / s.step)));
    for (int l = 0; l < layers; ++l) {
      const double d = s.gap + s.depth * l / std::max(1, layers - 1);
      for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double u = g.a.first + t * (g.b.first - g.a.first) + d * g.n.first;
        const double v = g.a.second + t * (g.b.second - g.a.second) + d * g.n.second;
        out.push_back(toMap(c, u, v));
      }
      // Convex corner to the next segment: fill the wedge between the two offsets.
      if (si + 1 < segs.size()) {
        const auto & h = segs[si + 1];
        const double a0 = std::atan2(g.n.second, g.n.first), a1 = std::atan2(h.n.second, h.n.first);
        const double da = std::remainder(a1 - a0, 2.0 * M_PI);
        const int m = std::max(1, static_cast<int>(std::ceil(std::abs(da) * d / s.step)));
        for (int j = 1; j < m; ++j) {
          const double a = a0 + da * j / m;
          out.push_back(toMap(c, g.b.first + d * std::cos(a), g.b.second + d * std::sin(a)));
        }
      }
    }
  }
  return out;
}

std::vector<Point> contactLine(const Contact & c, const MarkShape & s)
{
  std::vector<Point> out;
  const double d = s.gap + s.depth / 2.0;
  const auto segs = frontOutline(s.body, c.side);
  for (size_t i = 0; i < segs.size(); ++i) {
    const auto & g = segs[i];
    if (i == 0) out.push_back(toMap(c, g.a.first + d * g.n.first, g.a.second + d * g.n.second));
    out.push_back(toMap(c, g.b.first + d * g.n.first, g.b.second + d * g.n.second));
  }
  return out;
}

double FeltObstacle::distance(double x, double y) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & c : contacts) {
    for (const auto & [px, py] : c.marks) best = std::min(best, std::hypot(px - x, py - y));
  }
  return best;
}

size_t FeltObstacle::markCount() const
{
  size_t n = 0;
  for (const auto & c : contacts) n += c.marks.size();
  return n;
}

size_t FeltObstacle::totalMarks() const
{
  size_t n = 0;
  for (const auto & c : contacts) n += c.total;
  return n;
}

size_t FeltObstacle::erase(const std::function<bool(double, double)> & inside)
{
  size_t n = 0;
  for (auto & c : contacts) {
    const auto before = c.marks.size();
    c.marks.erase(std::remove_if(c.marks.begin(), c.marks.end(), [&](const Point & p) { return inside(p.first, p.second); }),
                  c.marks.end());
    n += before - c.marks.size();
  }
  return n;
}

bool insideBody(const Body & b, double x, double y, double yaw, double px, double py)
{
  const double dx = px - x, dy = py - y, cs = std::cos(yaw), sn = std::sin(yaw);
  const double u = dx * cs + dy * sn, v = std::abs(-dx * sn + dy * cs);
  const double ch = std::clamp(b.chamfer, 0.0, std::min(b.half_width, b.front));
  return u >= -b.rear && u <= b.front && v <= b.half_width && u + v <= b.front + b.half_width - ch;
}

std::pair<size_t, size_t> jogPath(std::vector<PathPose> & poses, size_t from, size_t c, double offset, double full,
                                  double ramp)
{
  if (poses.empty() || c >= poses.size() || from > c) return {1, 0};
  // Signed distance along the path from pose c.
  std::vector<double> s(poses.size(), 0.0);
  for (size_t i = c + 1; i < poses.size(); ++i)
    s[i] = s[i - 1] + std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
  for (size_t i = c; i-- > 0;) s[i] = s[i + 1] - std::hypot(poses[i + 1].x - poses[i].x, poses[i + 1].y - poses[i].y);
  size_t first = poses.size(), last = 0;
  const auto orig = poses;
  for (size_t i = from; i < poses.size(); ++i) {
    const double d = std::abs(s[i]);
    if (d >= full + ramp) continue;
    // Smooth (cosine) fade: FTC follows it without a kink.
    const double w = d <= full ? 1.0 : 0.5 * (1.0 + std::cos(M_PI * (d - full) / ramp));
    poses[i].x = orig[i].x - offset * w * std::sin(orig[i].yaw);
    poses[i].y = orig[i].y + offset * w * std::cos(orig[i].yaw);
    first = std::min(first, i);
    last = std::max(last, i);
  }
  if (first > last) return {1, 0};
  // Headings along the new line (one pose beyond the changed range on each side too).
  const size_t a = first > from ? first - 1 : first, b = std::min(poses.size() - 1, last + 1);
  for (size_t i = a; i <= b; ++i) {
    const auto & p = poses[i == 0 ? 0 : i - 1];
    const auto & n = poses[i + 1 < poses.size() ? i + 1 : i];
    if (std::hypot(n.x - p.x, n.y - p.y) > 1e-6) poses[i].yaw = std::atan2(n.y - p.y, n.x - p.x);
  }
  return {first, last};
}

bool sameContact(const Contact & a, const Contact & b, double max_dist, double max_dyaw)
{
  return a.side == b.side && std::hypot(a.x - b.x, a.y - b.y) <= max_dist &&
         std::abs(std::remainder(a.yaw - b.yaw, 2.0 * M_PI)) <= max_dyaw;
}

}  // namespace open_mower_next::mower_logic
