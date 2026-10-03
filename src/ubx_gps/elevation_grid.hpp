// Garden elevation from RTK fixed heights: a square grid (default 0.2 m) in the
// map frame with the mean height and spread per cell, kept in RAM and written
// as a whole CSV (x,y,z,n,std) now and then for import into CAD. Reloads its
// own file so the averages keep improving across restarts. No ROS here
// (unit-tested).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>

namespace open_mower_next::ubx_gps
{

class ElevationGrid
{
public:
  // max_samples per cell: a full cell takes no more, so a robot standing still
  // (docked) neither grows its weight forever nor keeps the file dirty.
  explicit ElevationGrid(double cell = 0.2, uint32_t max_samples = 100) : cell_(cell), max_samples_(max_samples) {}

  double cell() const { return cell_; }
  size_t size() const { return cells_.size(); }
  bool dirty() const { return dirty_; }

  // Cell index of a map position: cell (i, j) covers [i*cell, (i+1)*cell).
  std::pair<int32_t, int32_t> index(double x, double y) const
  {
    return {static_cast<int32_t>(std::floor(x / cell_)), static_cast<int32_t>(std::floor(y / cell_))};
  }

  // False if the cell is full.
  bool add(double x, double y, double z)
  {
    Cell & c = cells_[index(x, y)];
    if (c.n >= max_samples_) return false;
    // Welford: mean and spread without large sums.
    ++c.n;
    const double d = z - c.mean;
    c.mean += d / c.n;
    c.m2 += d * (z - c.mean);
    dirty_ = true;
    return true;
  }

  // Mean height of the cell at (x, y); NAN if it has no samples.
  double mean(double x, double y) const
  {
    const auto it = cells_.find(index(x, y));
    return it == cells_.end() ? NAN : it->second.mean;
  }

  uint32_t count(double x, double y) const
  {
    const auto it = cells_.find(index(x, y));
    return it == cells_.end() ? 0 : it->second.n;
  }

  // Cells with fewer than min_samples are left out. Writes to path + ".tmp" and
  // renames, so a reader (or a power cut) never sees half a file.
  bool save(const std::string & path, const std::string & header, uint32_t min_samples = 1)
  {
    const std::string tmp = path + ".tmp";
    {
      std::ofstream f(tmp, std::ios::trunc);
      if (!f) return false;
      if (!header.empty()) f << header;
      f << "x,y,z,n,std\n";
      char line[96];
      for (const auto & [k, c] : cells_) {
        if (c.n < min_samples) continue;
        const double sd = c.n > 1 ? std::sqrt(c.m2 / (c.n - 1)) : 0.0;
        std::snprintf(line, sizeof(line), "%.2f,%.2f,%.4f,%u,%.4f\n", (k.first + 0.5) * cell_,
                      (k.second + 0.5) * cell_, c.mean, c.n, sd);
        f << line;
      }
      if (!f.flush()) return false;
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) return false;
    dirty_ = false;
    return true;
  }

  // Merges a file written by save() (same cell size); '#' lines are skipped.
  // Returns the number of cells read, -1 if the file can't be opened.
  int load(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) return -1;
    std::string line;
    int read = 0;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#' || line[0] == 'x') continue;
      std::istringstream s(line);
      double x, y, z, sd;
      uint32_t n;
      char c1, c2, c3, c4;
      if (!(s >> x >> c1 >> y >> c2 >> z >> c3 >> n >> c4 >> sd) || n == 0) continue;
      Cell & c = cells_[index(x, y)];
      // Merge two (n, mean, M2) sets (Chan et al.).
      const double m2 = sd * sd * (n > 1 ? n - 1 : 0);
      const uint32_t total = c.n + n;
      const double d = z - c.mean;
      c.mean += d * n / total;
      c.m2 += m2 + d * d * c.n * n / total;
      c.n = total;
      ++read;
    }
    return read;
  }

private:
  struct Cell
  {
    uint32_t n = 0;
    double mean = 0.0;
    double m2 = 0.0;
  };

  double cell_;
  uint32_t max_samples_;
  std::map<std::pair<int32_t, int32_t>, Cell> cells_;  // sorted: the file reads row by row
  bool dirty_ = false;
};

}  // namespace open_mower_next::ubx_gps
