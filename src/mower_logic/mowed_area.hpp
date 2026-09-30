#pragma once
// Mowed area statistics for the web UI: this charge, the last few charges and
// the total, in m². The area is the metres of pass the mower followed times the
// lane spacing (coverage tool_width * (1 - overlap)); the resume backtrack and
// skipped pieces don't count. A charge ends when the mower arrives on the charger.

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace open_mower_next::mower_logic
{

class MowedArea
{
public:
  explicit MowedArea(size_t history = 3) : history_size_(history) {}

  void add(double m2);
  // Arrived on the charger: this charge moves into the history (only if
  // something was mowed, so a test dock keeps the history).
  void docked();

  double charge() const;
  std::deque<double> history() const;  // newest first
  double total() const;
  // Changes on every roll into the history (the caller saves at once then).
  uint64_t rolls() const;

  // {"charge":..,"last":[..],"total":..}, m² to 0.1
  std::string json() const;
  // Text, one "key value..." per line.
  std::string serialize() const;
  bool restore(const std::string & text);  // false (unchanged) on bad text

private:
  mutable std::mutex mutex_;
  size_t history_size_;
  double charge_ = 0.0, total_ = 0.0;
  std::deque<double> history_;
  uint64_t rolls_ = 0;
};

}  // namespace open_mower_next::mower_logic
