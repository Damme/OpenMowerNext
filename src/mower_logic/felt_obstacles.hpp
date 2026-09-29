#pragma once
// What the mower learns by bumping into things. The Worx bumper only says THAT
// the front touched something, not where. One bump = one contact: the part of
// the front outline that can have touched, marked as a thin band just outside
// the bumper. Contacts close together form one obstacle, and its shape is
// whatever the robot has felt of it so far - like a robot vacuum feeling its way
// around a chair, not a fixed-size blob.
//
// Pure geometry (no ROS), so it can be unit tested.

#include <cstdint>
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
  std::vector<Point> marks;  // filled by contactMarks()
};

// Band points of a contact, map frame: the front outline (front face, the cut
// corners and 5 cm of the sides behind them) offset outwards by gap..gap+depth.
std::vector<Point> contactMarks(const Contact & c, const MarkShape & s);

// Middle line of the band (map frame), for drawing.
std::vector<Point> contactLine(const Contact & c, const MarkShape & s);

struct FeltObstacle
{
  int id = 0;
  std::vector<Contact> contacts;

  // Distance from (x, y) to the nearest mark (infinity without contacts).
  double distance(double x, double y) const;
  size_t markCount() const;
};

// Is a new contact a repeat of an existing one (nearly the same pose)? Then it
// replaces it instead of piling up bands at one spot.
bool sameContact(const Contact & a, const Contact & b, double max_dist = 0.05, double max_dyaw = 0.26);

}  // namespace open_mower_next::mower_logic
