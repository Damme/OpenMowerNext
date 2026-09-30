#include "mower_logic/mowed_area.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace open_mower_next::mower_logic
{

namespace
{
double r1(double v)
{
  return std::round(v * 10.0) / 10.0;
}
}  // namespace

void MowedArea::add(double m2)
{
  if (!(m2 > 0.0) || !std::isfinite(m2)) return;
  std::lock_guard<std::mutex> l(mutex_);
  charge_ += m2;
  total_ += m2;
}

void MowedArea::docked()
{
  std::lock_guard<std::mutex> l(mutex_);
  if (charge_ < 0.05) return;
  history_.push_front(charge_);
  while (history_.size() > history_size_) history_.pop_back();
  charge_ = 0.0;
  ++rolls_;
}

double MowedArea::charge() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return charge_;
}

std::deque<double> MowedArea::history() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return history_;
}

double MowedArea::total() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return total_;
}

uint64_t MowedArea::rolls() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return rolls_;
}

std::string MowedArea::json() const
{
  std::lock_guard<std::mutex> l(mutex_);
  std::ostringstream s;
  s << "{\"charge\":" << r1(charge_) << ",\"last\":[";
  for (size_t i = 0; i < history_.size(); ++i) s << (i ? "," : "") << r1(history_[i]);
  s << "],\"total\":" << r1(total_) << "}";
  return s.str();
}

std::string MowedArea::serialize() const
{
  std::lock_guard<std::mutex> l(mutex_);
  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << "total " << total_ << "\ncharge " << charge_ << "\nlast";
  for (double h : history_) s << ' ' << h;
  s << '\n';
  return s.str();
}

bool MowedArea::restore(const std::string & text)
{
  std::istringstream in(text);
  double charge = 0.0, total = 0.0;
  std::deque<double> history;
  bool have_total = false;
  for (std::string line; std::getline(in, line);) {
    std::istringstream ls(line);
    std::string key;
    if (!(ls >> key)) continue;
    if (key == "total") {
      if (!(ls >> total)) return false;
      have_total = true;
    } else if (key == "charge") {
      if (!(ls >> charge)) return false;
    } else if (key == "last") {
      for (double v; ls >> v;) {
        if (history.size() < history_size_) history.push_back(v);
      }
    }
  }
  if (!have_total || !std::isfinite(total) || !std::isfinite(charge) || total < 0 || charge < 0) return false;
  std::lock_guard<std::mutex> l(mutex_);
  total_ = total;
  charge_ = charge;
  history_ = std::move(history);
  return true;
}

}  // namespace open_mower_next::mower_logic
