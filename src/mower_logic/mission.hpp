#pragma once
// Mowing progress: which operation areas, their planned passes, and how far
// along the current pass the mower got. Survives pauses (GPS loss, emergency,
// charging) so mowing resumes where it stopped, like the ROS1 MowingBehavior.

#include "open_mower_next/msg/coverage_path.hpp"

#include <nav_msgs/msg/path.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace open_mower_next::mower_logic
{

class Mission
{
public:
  struct Pass
  {
    nav_msgs::msg::Path path;  // remaining part to mow (from the resume index)
    bool is_outline = false;
    size_t area_index = 0, pass_index = 0, start_index = 0;
  };

  // Start (or restart) a mission over these areas; clears progress.
  void begin(const std::vector<std::string> & area_ids);
  void clear();
  bool active() const;

  // Current area without a plan yet?
  std::optional<std::string> areaNeedingPlan() const;
  void setPlan(const std::vector<open_mower_next::msg::CoveragePath> & passes);
  void skipArea();  // also used when planning fails

  // The pass to mow next (remaining part), nullopt when the mission is done.
  // resume_backtrack_m re-mows a little before the stop point.
  std::optional<Pass> currentPass(double resume_backtrack_m = 0.5) const;

  // Progress reports while following the current pass.
  void setPoseIndex(size_t absolute_index);
  size_t poseIndex() const;
  void passDone();
  // Returns true when the pass was skipped after too many failures.
  bool passFailed(int max_attempts);
  void skipPass();

  std::string summary() const;
  size_t areaCount() const;

private:
  mutable std::mutex mutex_;
  std::vector<std::string> areas_;
  std::vector<open_mower_next::msg::CoveragePath> passes_;
  bool planned_ = false;
  size_t area_ = 0, pass_ = 0, pose_ = 0;
  int attempts_ = 0;
  bool active_ = false;
};

}  // namespace open_mower_next::mower_logic
