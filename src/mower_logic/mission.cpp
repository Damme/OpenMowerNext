#include "mower_logic/mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>

namespace open_mower_next::mower_logic
{

void Mission::begin(const std::vector<std::string> & area_ids)
{
  std::lock_guard<std::mutex> l(mutex_);
  ++generation_;
  areas_ = area_ids;
  passes_.clear();
  planned_ = false;
  area_ = pass_ = pose_ = 0;
  attempts_ = 0;
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
  restore_.reset();
  active_ = !areas_.empty();
}

void Mission::clear()
{
  std::lock_guard<std::mutex> l(mutex_);
  ++generation_;
  areas_.clear();
  passes_.clear();
  planned_ = false;
  area_ = pass_ = pose_ = 0;
  attempts_ = 0;
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
  restore_.reset();
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

std::string Mission::setPlan(const std::vector<open_mower_next::msg::CoveragePath> & passes)
{
  std::lock_guard<std::mutex> l(mutex_);
  passes_ = passes;
  fingerprint_ = fingerprint(passes);
  planned_ = true;
  pass_ = pose_ = 0;
  attempts_ = 0;
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
  if (!restore_) return {};
  const Restore r = *restore_;
  restore_.reset();
  std::ostringstream s;
  if (r.fingerprint == fingerprint_ && r.pass < passes_.size()) {
    pass_ = r.pass;
    pose_ = std::min(r.pose, passes_[pass_].path.poses.size());
    s << "restored position: pass " << (pass_ + 1) << "/" << passes_.size() << ", pose " << pose_;
  } else {
    s << "saved position discarded (the plan changed since it was saved): area starts over";
  }
  return s.str();
}

void Mission::editPlan(const std::function<void(std::vector<open_mower_next::msg::CoveragePath> &)> & fn)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (planned_) fn(passes_);
}

bool Mission::editCurrentPass(const std::function<void(nav_msgs::msg::Path &, size_t)> & fn)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!active_ || !planned_ || pass_ >= passes_.size()) return false;
  fn(passes_[pass_].path, pose_);
  return true;
}

size_t Mission::passKey() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return (static_cast<size_t>(generation_.load()) << 40) ^ (area_ << 20) ^ pass_ ^ (active_ ? 0 : (size_t{1} << 60));
}

void Mission::skipArea()
{
  ++generation_;
  std::lock_guard<std::mutex> l(mutex_);
  ++area_;
  passes_.clear();
  planned_ = false;
  restore_.reset();
  pass_ = pose_ = 0;
  attempts_ = 0;
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
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
  while (!no_backtrack_ && start > 0 && back < resume_backtrack_m) {
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
  if (absolute_index > pose_) {
    pose_ = absolute_index;
    no_backtrack_ = false;
  }
}

void Mission::continueAt(size_t absolute_index)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (absolute_index > pose_) pose_ = absolute_index;
  no_backtrack_ = true;
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
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
  if (pass_ >= passes_.size()) {
    ++area_;
    passes_.clear();
    planned_ = false;
    pass_ = 0;
    if (area_ >= areas_.size()) active_ = false;
  }
}

bool Mission::passFailed(int max_attempts, double skip_step_m, int max_continuation_steps)
{
  bool skip = false;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (no_backtrack_ && pass_ < passes_.size() && continuation_steps_ < max_continuation_steps) {
      // Continuation past an obstacle unreachable: try further along the pass.
      const double step = skip_step_m * static_cast<double>(1 << continuation_steps_++);
      const auto & poses = passes_[pass_].path.poses;
      double along = 0.0;
      while (pose_ + 3 < poses.size() && along < step) {
        const auto & a = poses[pose_].pose.position;
        const auto & b = poses[pose_ + 1].pose.position;
        along += std::hypot(b.x - a.x, b.y - a.y);
        ++pose_;
      }
      if (pose_ + 3 < poses.size()) return false;
    }
    skip = ++attempts_ >= max_attempts;
  }
  if (skip) passDone();
  return skip;
}

void Mission::skipPass()
{
  ++generation_;
  passDone();
}

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
  if (!planned_ && restore_) s << ", saved pass " << (restore_->pass + 1) << ", pose " << restore_->pose;
  if (attempts_) s << ", attempt " << (attempts_ + 1);
  return s.str();
}

bool Mission::skipPastPoint(double x, double y, double clearance_m, int max_bumps, bool count_bump)
{
  return skipPast([&](double px, double py) { return std::hypot(px - x, py - y) <= clearance_m; }, max_bumps,
                  count_bump);
}

bool Mission::skipPast(const std::function<bool(double, double)> & near_fn, int max_bumps, bool count_bump)
{
  // near_fn may take other locks (the context's obstacles): not called under mutex_.
  std::vector<std::pair<double, double>> pts;
  size_t pass = 0, from = 0;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!active_ || !planned_ || pass_ >= passes_.size()) return false;
    if (count_bump && ++bumps_ > max_bumps) {
      pts.clear();
    } else {
      pass = pass_;
      from = pose_;
      for (const auto & p : passes_[pass_].path.poses) pts.emplace_back(p.pose.position.x, p.pose.position.y);
    }
  }
  std::optional<size_t> resume;
  bool near = false;
  for (size_t i = from; i + 2 < pts.size(); ++i) {
    if (near_fn(pts[i].first, pts[i].second)) {
      near = true;
    } else if (near) {
      resume = i;
      break;
    }
  }
  if (resume) {
    std::lock_guard<std::mutex> l(mutex_);
    if (pass_ == pass && active_) {
      pose_ = *resume;
      no_backtrack_ = true;
      return true;
    }
    return false;  // the mission moved on meanwhile (operator skip)
  }
  passDone();  // obstacle at the end of the pass, or bumped too often
  return false;
}

bool Mission::countBump(int max_bumps)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!active_ || !planned_ || pass_ >= passes_.size()) return false;
    if (++bumps_ <= max_bumps) return true;
  }
  passDone();
  return false;
}

namespace
{
constexpr const char * kMagic = "openmower_mission 1";
}

std::string Mission::fingerprint(const std::vector<open_mower_next::msg::CoveragePath> & passes)
{
  std::ostringstream s;
  s << passes.size();
  auto r = [](double v) { return static_cast<long>(std::lround(v * 10.0)); };
  for (const auto & p : passes) {
    s << ";" << p.path.poses.size() << (p.is_outline ? "o" : "m");
    if (!p.is_outline && !p.path.poses.empty()) {
      const auto & a = p.path.poses.front().pose.position;
      const auto & b = p.path.poses.back().pose.position;
      s << r(a.x) << "," << r(a.y) << "," << r(b.x) << "," << r(b.y);
    }
  }
  // FNV-1a: short and stable across builds.
  uint64_t h = 1469598103934665603ull;
  for (const unsigned char c : s.str()) {
    h ^= c;
    h *= 1099511628211ull;
  }
  std::ostringstream hex;
  hex << std::hex << h;
  return hex.str();
}

std::string Mission::serialize() const
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!active_ || area_ >= areas_.size()) return {};
  std::ostringstream s;
  s << kMagic << "\nareas ";
  for (size_t i = 0; i < areas_.size(); ++i) s << (i ? "," : "") << areas_[i];
  s << "\narea " << area_ << "\n";
  if (planned_) {
    s << "pass " << pass_ << "\npose " << pose_ << "\nplan " << fingerprint_ << "\n";
  } else if (restore_) {
    s << "pass " << restore_->pass << "\npose " << restore_->pose << "\nplan " << restore_->fingerprint << "\n";
  }
  return s.str();
}

bool Mission::restore(const std::string & text)
{
  std::istringstream in(text);
  std::string line;
  if (!std::getline(in, line) || line != kMagic) return false;
  std::vector<std::string> areas;
  std::optional<size_t> area;
  Restore r;
  bool has_pass = false;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string key, value;
    ls >> key >> value;
    try {
      if (key == "areas") {
        std::stringstream as(value);
        for (std::string id; std::getline(as, id, ',');) {
          if (!id.empty()) areas.push_back(id);
        }
      } else if (key == "area") {
        area = std::stoul(value);
      } else if (key == "pass") {
        r.pass = std::stoul(value);
        has_pass = true;
      } else if (key == "pose") {
        r.pose = std::stoul(value);
      } else if (key == "plan") {
        r.fingerprint = value;
      }
    } catch (const std::exception &) {
      return false;
    }
  }
  if (areas.empty() || !area || *area >= areas.size()) return false;
  std::lock_guard<std::mutex> l(mutex_);
  ++generation_;
  areas_ = areas;
  area_ = *area;
  passes_.clear();
  fingerprint_.clear();
  planned_ = false;
  pass_ = pose_ = 0;
  attempts_ = 0;
  bumps_ = 0;
  continuation_steps_ = 0;
  no_backtrack_ = false;
  if (has_pass && !r.fingerprint.empty()) {
    restore_ = r;
  } else {
    restore_.reset();
  }
  active_ = true;
  return true;
}

}  // namespace open_mower_next::mower_logic
