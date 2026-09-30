#pragma once
// Mowing progress: which operation areas, their planned passes, and how far
// along the current pass the mower got. Survives pauses (GPS loss, emergency,
// charging) so mowing resumes where it stopped, like the ROS1 MowingBehavior.

#include "open_mower_next/msg/coverage_path.hpp"

#include <nav_msgs/msg/path.hpp>

#include <atomic>
#include <functional>
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
  // Returns a note when a restored position was applied or discarded (else empty).
  std::string setPlan(const std::vector<open_mower_next::msg::CoveragePath> & passes);
  void skipArea();  // also used when planning fails
  // Modify the current area's plan in place (edge corrections); progress is kept.
  void editPlan(const std::function<void(std::vector<open_mower_next::msg::CoveragePath> &)> & fn);
  // The same for the current pass alone (sidestepping an obstacle); fn gets the
  // progress index. False without a current pass.
  bool editCurrentPass(const std::function<void(nav_msgs::msg::Path &, size_t pose_index)> & fn);
  // Changes whenever the current pass changes (area/pass), for per-pass state.
  size_t passKey() const;

  // The pass to mow next (remaining part), nullopt when the mission is done.
  // resume_backtrack_m re-mows a little before the stop point.
  std::optional<Pass> currentPass(double resume_backtrack_m = 0.5) const;

  // Progress reports while following the current pass. Returns the metres of
  // pass newly covered (0 when going back over the resume backtrack).
  double setPoseIndex(size_t absolute_index);
  // Continue the current pass at this pose (after a segment cut before a
  // corner the body can't drive), without the resume backtrack.
  void continueAt(size_t absolute_index);
  size_t poseIndex() const;
  void passDone();
  // Returns true when the pass was skipped after too many failures. A failed
  // continuation past an obstacle (no reachable start there, e.g. a dead end
  // behind the obstacle) moves further along the pass instead - 1, 2, 4, 8,
  // 16 m - and only then counts as a failure of the pass.
  bool passFailed(int max_attempts, double skip_step_m = 1.0, int max_continuation_steps = 5);
  void skipPass();
  // Bumped into an obstacle at (x, y) while on the current pass: continue at
  // the first pose after it that is more than clearance_m away, without the
  // resume backtrack. Skips the pass after max_bumps bumps on it.
  // Returns false when the rest of the pass was skipped.
  // count_bump false: a planned skip around a known obstacle (no bump).
  bool skipPastPoint(double x, double y, double clearance_m, int max_bumps, bool count_bump = true);
  // The same for an obstacle of any shape: near(x, y) is true for poses too
  // close to it. Continues at the first pose after the near ones.
  bool skipPast(const std::function<bool(double, double)> & near, int max_bumps, bool count_bump = true);
  // A bump that is handled without skipping (felt around, or in a transit):
  // counts towards max_bumps too. False (pass skipped) when there were too many.
  bool countBump(int max_bumps);

  std::string summary() const;

  // Persistence (Daniel: after a restart the mower must still know where it
  // stopped). Text, one "key value" per line; empty when there is no mission.
  std::string serialize() const;
  // Restores areas and the current area; the area is re-planned on demand and
  // the saved pass/pose applied only if the new plan has the same fingerprint
  // (else the area starts over). Returns false (mission unchanged) on bad text.
  bool restore(const std::string & text);
  // Pass count, pose count and outline flag per pass, and the mowing passes'
  // end points to 0.1 m (edge corrections only move outline poses).
  static std::string fingerprint(const std::vector<open_mower_next::msg::CoveragePath> & passes);
  // Changes on operator skips/resets (skipPass, skipArea, begin, clear): a
  // transit or pass started under an older generation is stale.
  unsigned generation() const { return generation_.load(); }
  size_t areaCount() const;

private:
  mutable std::mutex mutex_;
  std::vector<std::string> areas_;
  std::vector<open_mower_next::msg::CoveragePath> passes_;
  bool planned_ = false;
  size_t area_ = 0, pass_ = 0, pose_ = 0;
  int attempts_ = 0;
  int bumps_ = 0;               // on the current pass
  int continuation_steps_ = 0;  // unreachable continuation: skips so far
  bool no_backtrack_ = false;   // next resume starts exactly at pose_
  bool active_ = false;
  std::string fingerprint_;  // of passes_
  struct Restore
  {
    size_t pass = 0, pose = 0;
    std::string fingerprint;
  };
  std::optional<Restore> restore_;  // saved position waiting for the area's plan
  std::atomic<unsigned> generation_{0};
};

}  // namespace open_mower_next::mower_logic
