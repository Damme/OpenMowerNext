// UBX frame parser for the u-blox F9P: splits a byte stream into frames and
// decodes NAV-PVT, plus NAV-SAT, NAV-SIG, NAV-RELPOSNED, MON-RF and RXM-RTCM for
// the receiver statistics (gps_stats.hpp). No ROS here (unit-tested).
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
  bool diff_soln = false;   // flags.diffSoln: corrections applied
  uint8_t carr_soln = 0;    // 0 no RTK, 1 RTK float, 2 RTK fixed
  uint8_t num_sv = 0;
  double lon_deg = 0, lat_deg = 0;
  double height_m = 0;      // above the ellipsoid
  double h_msl_m = 0;       // above mean sea level (receiver's geoid model)
  double h_acc_m = 0, v_acc_m = 0;
  double g_speed = 0;       // m/s, ground speed
  double p_dop = 0;
  double corr_age_s = -1;   // flags3.lastCorrectionAge as its upper bound, -1 = not available
};

inline uint16_t u16(const uint8_t * p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }
inline int16_t i16(const uint8_t * p) { return static_cast<int16_t>(u16(p)); }
inline uint32_t u32(const uint8_t * p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }
inline int32_t i32(const uint8_t * p) { return static_cast<int32_t>(u32(p)); }

inline std::optional<NavPvt> parseNavPvt(const uint8_t * p, size_t len)
{
  if (len < 92) return std::nullopt;
  NavPvt n;
  n.itow_ms = u32(p);
  n.fix_type = p[20];
  n.fix_ok = p[21] & 0x01;
  n.diff_soln = p[21] & 0x02;
  n.carr_soln = p[21] >> 6;
  n.num_sv = p[23];
  n.lon_deg = i32(p + 24) * 1e-7;
  n.lat_deg = i32(p + 28) * 1e-7;
  n.height_m = i32(p + 32) * 1e-3;
  n.h_msl_m = i32(p + 36) * 1e-3;
  n.h_acc_m = u32(p + 40) * 1e-3;
  n.v_acc_m = u32(p + 44) * 1e-3;
  n.g_speed = i32(p + 60) * 1e-3;
  n.p_dop = u16(p + 76) * 0.01;
  static const double kAge[] = {-1, 1, 2, 5, 10, 15, 20, 30, 45, 60, 90, 120, 1e9};
  const unsigned age = (u16(p + 78) >> 1) & 0x0f;
  n.corr_age_s = age < 13 ? kAge[age] : -1;
  return n;
}

// gnssId: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS, 7 NavIC
struct SatInfo
{
  uint8_t gnss = 0, sv = 0;
  uint8_t cno = 0;          // dB-Hz, 0 = not tracked
  int8_t elev = 0;          // deg, outside -90..90 = unknown
  int16_t azim = 0;         // deg
  uint8_t quality = 0;      // 0 no signal .. 4 code locked .. 5-7 code and carrier locked
  bool used = false;        // in the navigation solution
  bool diff_corr = false;   // corrections available for it
};

inline std::optional<std::vector<SatInfo>> parseNavSat(const uint8_t * p, size_t len)
{
  if (len < 8 || p[4] != 1) return std::nullopt;
  const size_t n = p[5];
  if (len < 8 + 12 * n) return std::nullopt;
  std::vector<SatInfo> out(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * b = p + 8 + 12 * i;
    const uint32_t flags = u32(b + 8);
    out[i] = {b[0], b[1], b[2], static_cast<int8_t>(b[3]), i16(b + 4),
              static_cast<uint8_t>(flags & 0x07), static_cast<bool>(flags & 0x08), static_cast<bool>(flags & 0x40)};
  }
  return out;
}

// L1 band (L1C/A, E1, B1I/B1C, L1OF) or the second one (L2, E5b, B2I, L5)
inline bool isL1Signal(uint8_t gnss, uint8_t sig)
{
  switch (gnss) {
    case 0: case 1: case 6: return sig == 0;
    case 2: case 5: return sig <= 1;
    case 3: return sig <= 1 || sig == 5 || sig == 6;
    default: return false;
  }
}

struct SigInfo
{
  uint8_t gnss = 0, sv = 0, sig = 0;
  bool l1 = true;
  uint8_t cno = 0;
  uint8_t quality = 0;
  bool pr_used = false;     // pseudorange used in the solution
  bool cr_used = false;     // carrier range used (RTK)
};

inline std::optional<std::vector<SigInfo>> parseNavSig(const uint8_t * p, size_t len)
{
  if (len < 8 || p[4] != 0) return std::nullopt;
  const size_t n = p[5];
  if (len < 8 + 16 * n) return std::nullopt;
  std::vector<SigInfo> out(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * b = p + 8 + 16 * i;
    const uint16_t flags = u16(b + 10);
    out[i] = {b[0], b[1], b[2], isL1Signal(b[0], b[2]), b[6], b[7],
              static_cast<bool>(flags & 0x08), static_cast<bool>(flags & 0x10)};
  }
  return out;
}

struct RfBlock
{
  uint8_t id = 0;           // 0 L1, 1 L2/L5
  uint8_t jamming = 0;      // 0 unknown, 1 ok, 2 warning, 3 critical
  uint8_t ant_status = 0;   // 0 init, 1 don't know, 2 ok, 3 short, 4 open
  uint8_t ant_power = 0;    // 0 off, 1 on, 2 don't know
  uint16_t noise = 0;       // noisePerMS
  uint16_t agc = 0;         // 0..8191
  uint8_t jam_ind = 0;      // CW jamming 0..255
};

inline std::optional<std::vector<RfBlock>> parseMonRf(const uint8_t * p, size_t len)
{
  if (len < 4 || p[0] != 0) return std::nullopt;
  const size_t n = p[1];
  if (len < 4 + 24 * n) return std::nullopt;
  std::vector<RfBlock> out(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * b = p + 4 + 24 * i;
    out[i] = {b[0], static_cast<uint8_t>(b[1] & 0x03), b[2], b[3], u16(b + 12), u16(b + 14), b[16]};
  }
  return out;
}

struct RtcmInput
{
  uint16_t type = 0;
  uint16_t ref_station = 0;
  bool crc_failed = false;
  uint8_t used = 0;         // 0 unknown, 1 not used, 2 used
};

inline std::optional<RtcmInput> parseRxmRtcm(const uint8_t * p, size_t len)
{
  if (len < 8 || p[0] != 2) return std::nullopt;
  return RtcmInput{u16(p + 6), u16(p + 4), static_cast<bool>(p[1] & 0x01), static_cast<uint8_t>((p[1] >> 1) & 0x03)};
}

struct RelPosNed
{
  uint16_t ref_station = 0;
  double length_m = 0;      // baseline to the base station
  double n_m = 0, e_m = 0, d_m = 0;
  bool valid = false;       // relPosValid
  uint8_t carr_soln = 0;
  bool ref_pos_miss = false, ref_obs_miss = false;  // base position / observations missing
};

inline std::optional<RelPosNed> parseNavRelPosNed(const uint8_t * p, size_t len)
{
  if (len < 64 || p[0] != 1) return std::nullopt;
  const uint32_t flags = u32(p + 60);
  RelPosNed r;
  r.ref_station = u16(p + 2);
  r.n_m = i32(p + 8) * 0.01 + static_cast<int8_t>(p[32]) * 1e-4;
  r.e_m = i32(p + 12) * 0.01 + static_cast<int8_t>(p[33]) * 1e-4;
  r.d_m = i32(p + 16) * 0.01 + static_cast<int8_t>(p[34]) * 1e-4;
  r.length_m = i32(p + 20) * 0.01 + static_cast<int8_t>(p[35]) * 1e-4;
  r.valid = flags & 0x04;
  r.carr_soln = (flags >> 3) & 0x03;
  r.ref_pos_miss = flags & 0x40;
  r.ref_obs_miss = flags & 0x80;
  return r;
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
