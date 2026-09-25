#include "mower_logic/mission.hpp"

#include <cmath>
#include <sstream>

namespace open_mower_next::mower_logic
{

void Mission::begin(const std::vector<std::string> & area_ids)
{
  std::lock_guard<std::mutex> l(mutex_);
  areas_ = area_ids;
  passes_.clear();
  planned_ = false;
  area_ = pass_ = pose_ = 0;
  attempts_ = 0;
  active_ = !areas_.empty();
}

void Mission::clear()
{
  std::lock_guard<std::mutex> l(mutex_);
  areas_.clear();
  passes_.clear();
  planned_ = false;
  area_ = pass_ = pose_ = 0;
  attempts_ = 0;
  active_ = false;
}

bool Mission::active() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return active_;
}

std::optional<std::string> Mission::areaNeedingPlan() const
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!active_ || area_ >= areas_.size() || planned_) return std::nullopt;
  return areas_[area_];
}

void Mission::setPlan(const std::vector<open_mower_next::msg::CoveragePath> & passes)
{
  std::lock_guard<std::mutex> l(mutex_);
  passes_ = passes;
  planned_ = true;
  pass_ = pose_ = 0;
  attempts_ = 0;
}

void Mission::skipArea()
{
  std::lock_guard<std::mutex> l(mutex_);
  ++area_;
  passes_.clear();
  planned_ = false;
  pass_ = pose_ = 0;
  attempts_ = 0;
  if (area_ >= areas_.size()) active_ = false;
}

std::optional<Mission::Pass> Mission::currentPass(double resume_backtrack_m) const
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!active_ || !planned_ || pass_ >= passes_.size()) return std::nullopt;
  const auto & full = passes_[pass_].path;
  if (full.poses.size() < 2) return std::nullopt;
  // Back up along the path a little so the resume overlaps the stop point.
  size_t start = std::min(pose_, full.poses.size() - 2);
  double back = 0.0;
  while (start > 0 && back < resume_backtrack_m) {
    const auto & a = full.poses[start].pose.position;
    const auto & b = full.poses[start - 1].pose.position;
    back += std::hypot(a.x - b.x, a.y - b.y);
    --start;
  }
  Pass p;
  p.path.header = full.header;
  p.path.poses.assign(full.poses.begin() + static_cast<long>(start), full.poses.end());
  p.is_outline = passes_[pass_].is_outline;
  p.area_index = area_;
  p.pass_index = pass_;
  p.start_index = start;
  return p;
}

void Mission::setPoseIndex(size_t absolute_index)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (absolute_index > pose_) pose_ = absolute_index;
}

size_t Mission::poseIndex() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return pose_;
}

void Mission::passDone()
{
  std::lock_guard<std::mutex> l(mutex_);
  ++pass_;
  pose_ = 0;
  attempts_ = 0;
  if (pass_ >= passes_.size()) {
    ++area_;
    passes_.clear();
    planned_ = false;
    pass_ = 0;
    if (area_ >= areas_.size()) active_ = false;
  }
}

bool Mission::passFailed(int max_attempts)
{
  bool skip = false;
  {
    std::lock_guard<std::mutex> l(mutex_);
    skip = ++attempts_ >= max_attempts;
  }
  if (skip) passDone();
  return skip;
}

void Mission::skipPass() { passDone(); }

size_t Mission::areaCount() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return areas_.size();
}

std::string Mission::summary() const
{
  std::lock_guard<std::mutex> l(mutex_);
  std::ostringstream s;
  if (!active_) return "no mission";
  s << "area " << (area_ + 1) << "/" << areas_.size();
  if (area_ < areas_.size()) s << " (" << areas_[area_] << ")";
  if (planned_) s << ", pass " << (pass_ + 1) << "/" << passes_.size() << ", pose " << pose_;
  if (attempts_) s << ", attempt " << (attempts_ + 1);
  return s.str();
}

}  // namespace open_mower_next::mower_logic
