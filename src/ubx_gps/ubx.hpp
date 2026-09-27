// UBX frame parser for the u-blox F9P: splits a byte stream into frames and
// decodes NAV-PVT. No ROS here (unit-tested).
#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

namespace open_mower_next::ubx_gps
{

struct NavPvt
{
  uint32_t itow_ms = 0;
  uint8_t fix_type = 0;     // 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
  bool fix_ok = false;      // flags.gnssFixOK
  uint8_t carr_soln = 0;    // 0 no RTK, 1 RTK float, 2 RTK fixed
  uint8_t num_sv = 0;
  double lon_deg = 0, lat_deg = 0;
  double height_m = 0;      // above the ellipsoid
  double h_acc_m = 0, v_acc_m = 0;
};

inline uint32_t u32(const uint8_t * p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }
inline int32_t i32(const uint8_t * p) { return static_cast<int32_t>(u32(p)); }

inline std::optional<NavPvt> parseNavPvt(const uint8_t * p, size_t len)
{
  if (len < 92) return std::nullopt;
  NavPvt n;
  n.itow_ms = u32(p);
  n.fix_type = p[20];
  n.fix_ok = p[21] & 0x01;
  n.carr_soln = p[21] >> 6;
  n.num_sv = p[23];
  n.lon_deg = i32(p + 24) * 1e-7;
  n.lat_deg = i32(p + 28) * 1e-7;
  n.height_m = i32(p + 32) * 1e-3;
  n.h_acc_m = u32(p + 40) * 1e-3;
  n.v_acc_m = u32(p + 44) * 1e-3;
  return n;
}

// Feed bytes; calls on_frame(cls, id, payload, len) for every frame with a
// valid checksum. Anything else in the stream (NMEA, RTCM) is skipped.
class FrameParser
{
public:
  using Callback = std::function<void(uint8_t cls, uint8_t id, const uint8_t * payload, size_t len)>;
  explicit FrameParser(Callback cb) : cb_(std::move(cb)) {}

  void feed(const uint8_t * data, size_t n)
  {
    buf_.insert(buf_.end(), data, data + n);
    size_t pos = 0;
    while (true) {
      while (pos + 1 < buf_.size() && !(buf_[pos] == 0xB5 && buf_[pos + 1] == 0x62)) ++pos;
      if (pos + 6 > buf_.size()) break;
      const size_t len = buf_[pos + 4] | buf_[pos + 5] << 8;
      if (len > kMaxPayload) {
        ++pos;
        continue;
      }
      if (pos + 8 + len > buf_.size()) break;
      uint8_t a = 0, b = 0;
      for (size_t i = pos + 2; i < pos + 6 + len; ++i) {
        a += buf_[i];
        b += a;
      }
      if (a == buf_[pos + 6 + len] && b == buf_[pos + 7 + len]) {
        cb_(buf_[pos + 2], buf_[pos + 3], buf_.data() + pos + 6, len);
        pos += 8 + len;
      } else {
        ++pos;
      }
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(std::min(pos, buf_.size())));
  }

private:
  static constexpr size_t kMaxPayload = 4096;
  Callback cb_;
  std::vector<uint8_t> buf_;
};

}  // namespace open_mower_next::ubx_gps
