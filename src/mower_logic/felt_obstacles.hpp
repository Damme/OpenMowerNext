#pragma once
// What the mower learns by bumping into things. The Worx bumper only says THAT
// the front touched something, not where. One bump = one contact: the part of
// the front outline that can have touched, marked as a thin band just outside
// the bumper. Contacts close together form one obstacle. Wherever the body
// drives later, the marks under it are erased (nothing can be there), so what
// is left maps the real object - or nothing, for a false detection (Daniel
// 2026-09-29: "trying to map the object itself if it's really there").
//
// Pure geometry (no ROS), so it can be unit tested.

#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace open_mower_next::mower_logic
{

using Point = std::pair<double, double>;

// The mower's footprint in base_link (x forward): rear at -rear, front at +front,
// front corners cut at 45 deg by `chamfer` (the real ones are round).
struct Body
{
  double front = 0.47, rear = 0.11, half_width = 0.195, chamfer = 0.10;
};

struct MarkShape
{
  Body body;
  double gap = 0.02;    // m between the bumper and the band
  double depth = 0.10;  // m band depth (two costmap cells: a 1-cell line can slip between footprint checks)
  double step = 0.025;  // m between points (half a costmap cell: no holes)
};

struct Contact
{
  uint64_t id = 0;
  double x = 0, y = 0, yaw = 0;  // robot (base_link) pose at the bump, map frame
  // Which part of the front can have touched: 0 anywhere (driving straight),
  // +1 the left corner, -1 the right corner (arcing towards that side): the
  // cut corner, 5 cm of the side and the outer half of that side's front face.
  int side = 0;
  std::vector<Point> marks;  // filled by contactMarks(); erased where the body drove since
  size_t total = 0;          // marks at the bump
};

// Band points of a contact, map frame: the front outline (front face, the cut
// corners and 5 cm of the sides behind them) offset outwards by gap..gap+depth.
std::vector<Point> contactMarks(const Contact & c, const MarkShape & s);

// Middle line of the band (map frame), for drawing.
std::vector<Point> contactLine(const Contact & c, const MarkShape & s);

struct FeltObstacle
{
  // PENDING: one bump, not believed yet (tried again first). CONFIRMED: bumped
  // again at the same spot. GONE: the body drove through where it was felt (a
  // false detection, or it was moved) - kept to look at in the web UI.
  enum class State { PENDING, CONFIRMED, GONE };
  int id = 0;
  std::vector<Contact> contacts;
  State state = State::PENDING;

  // Distance from (x, y) to the nearest mark (infinity without marks).
  double distance(double x, double y) const;
  size_t markCount() const;
  size_t totalMarks() const;
  // Erase the marks for which inside(x, y) is true; returns how many.
  size_t erase(const std::function<bool(double, double)> & inside);
};

// Inside the mower's footprint at (x, y, yaw) (front corners cut)?
bool insideBody(const Body & b, double x, double y, double yaw, double px, double py);

// A pose of a path (map frame), for jogPath.
struct PathPose
{
  double x = 0, y = 0, yaw = 0;
};

// Shift a path sideways by offset (m, left of its direction positive) around
// pose c: fully within `full` m along the path on both sides, fading out over
// `ramp` m beyond; poses before `from` stay. Headings follow the new line.
// Returns the index range [first, last] that changed (first > last: none).
std::pair<size_t, size_t> jogPath(std::vector<PathPose> & poses, size_t from, size_t c, double offset, double full,
                                  double ramp);

// Is a new contact a repeat of an existing one (nearly the same pose)? Then it
// replaces it instead of piling up bands at one spot.
bool sameContact(const Contact & a, const Contact & b, double max_dist = 0.05, double max_dyaw = 0.26);

}  // namespace open_mower_next::mower_logic
