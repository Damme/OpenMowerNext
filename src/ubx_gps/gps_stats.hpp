// Receiver statistics for the web UI's GPS tab (published by ubx_gps as JSON on
// gps/stats): what the F9P sees now (satellites, signal strength, antenna, RTCM input)
// and how RTK held up since the start (time per solution type, fixed losses, how far
// the position was off when fixed came back, jumps between fixed samples), plus a
// two-hour history in 20 s buckets. No ROS here (unit-tested).
//
// Times: t = monotonic seconds (ages, durations), wall = epoch seconds (shown).
// Solution quality q: 0 no fix, 1 fix without RTK, 2 RTK float, 3 RTK fixed.
#pragma once

#include "ubx_gps/ubx.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <utility>
#include <vector>

namespace open_mower_next::ubx_gps
{

class GpsStats
{
public:
  using Json = nlohmann::json;

  static constexpr double kBucket = 20.0;     // s per history bucket
  static constexpr size_t kHistory = 360;     // buckets: 2 h
  static constexpr double kFresh = 5.0;       // s: NAV-SAT/SIG, MON-RF, RELPOSNED older than this are not shown
  static constexpr double kJumpSlack = 0.10;  // m: fixed-to-fixed movement allowed beyond gSpeed * dt
  static constexpr double kJumpMin = 0.15;    // m: smaller steps are never a jump
  static constexpr double kRefixSuspect = 0.30;  // m: re-fix offset beyond the distance driven

  static int quality(const NavPvt & p)
  {
    if (!p.fix_ok || p.fix_type < 2 || p.fix_type > 4) return 0;
    return p.carr_soln == 2 ? 3 : p.carr_soln == 1 ? 2 : 1;
  }

  void onPvt(const NavPvt & p, double t, double wall)
  {
    advance(t, wall);
    const int q = quality(p);
    if (!have_origin_ && q > 0) {
      have_origin_ = true;
      lat0_ = p.lat_deg;
      lon0_ = p.lon_deg;
    }
    const auto [e, n] = local(p);
    if (have_pvt_) {
      const double dt = std::clamp(t - pvt_t_, 0.0, 1.0);
      seconds_[last_q_] += dt;
      bucket_.seconds[last_q_] += dt;
      if (q == 3 && last_q_ == 3 && dt > 0) {
        const double d = std::hypot(e - fix_e_, n - fix_n_);
        const double allowed = std::max(p.g_speed, last_.g_speed) * dt + kJumpSlack;
        if (d > allowed && d > kJumpMin) {
          ++jumps_;
          addEvent({{"wall", wall}, {"kind", "jump"}, {"offset", r2(d)}, {"moved", r2(allowed - kJumpSlack)}});
        }
      }
      if (last_q_ == 3 && q < 3) {
        ++losses_;
        loss_t_ = t;
        loss_wall_ = wall;
        moved_ = 0;
        worst_ = q;
      } else if (last_q_ < 3 && q < 3 && loss_t_ >= 0) {
        moved_ += std::max(0.0, p.g_speed) * dt;
        worst_ = std::min(worst_, q);
      } else if (last_q_ < 3 && q == 3 && loss_t_ >= 0) {
        const double lost = t - loss_t_;
        const double offset = std::hypot(e - fix_e_, n - fix_n_);
        refix_s_.push_back(lost);
        if (refix_s_.size() > 50) refix_s_.pop_front();
        const bool suspect = offset - moved_ > kRefixSuspect;
        if (suspect) ++suspect_refix_;
        addEvent({{"wall", loss_wall_},
                  {"kind", "refix"},
                  {"lost", r1(lost)},
                  {"worst", worst_},
                  {"offset", r2(offset)},
                  {"moved", r2(moved_)},
                  {"suspect", suspect}});
        loss_t_ = -1;
      }
    }
    if (q == 3) {
      fix_e_ = e;
      fix_n_ = n;
    }
    bucket_.used_sum += p.num_sv;
    ++bucket_.used_n;
    last_ = p;
    last_q_ = q;
    pvt_t_ = t;
    have_pvt_ = true;
  }

  void onSat(std::vector<SatInfo> sats, double t)
  {
    sats_ = std::move(sats);
    sat_t_ = t;
    std::vector<int> cno;
    for (const auto & s : sats_) {
      if (s.cno > 0) cno.push_back(s.cno);
    }
    if (const double v = top4(cno); v > 0) {
      bucket_.cno_sum += v;
      ++bucket_.cno_n;
    }
  }

  void onSig(std::vector<SigInfo> sigs, double t)
  {
    sigs_ = std::move(sigs);
    sig_t_ = t;
    std::array<std::vector<int>, 2> cno;
    for (const auto & s : sigs_) {
      if (s.cno > 0) cno[s.l1 ? 0 : 1].push_back(s.cno);
    }
    for (int b = 0; b < 2; ++b) {
      if (const double v = top4(cno[b]); v > 0) {
        bucket_.band_sum[b] += v;
        ++bucket_.band_n[b];
      }
    }
  }

  void onRf(std::vector<RfBlock> rf, double t)
  {
    rf_ = std::move(rf);
    rf_t_ = t;
  }

  void onRelPos(const RelPosNed & r, double t)
  {
    relpos_ = r;
    relpos_t_ = t;
  }

  void onRtcm(const RtcmInput & m, double t)
  {
    rotateRtcm(t);
    auto & c = rtcm_[m.type];
    if (c.total > 0) {  // typical gap; an outage counts at most twice the usual interval
      const double gap = t - c.t;
      c.interval = c.interval > 0 ? 0.7 * c.interval + 0.3 * std::min(gap, 2 * c.interval) : gap;
    }
    ++c.total;
    ++c.minute;
    if (m.crc_failed) ++c.crc;
    c.used = m.used;
    c.ref = m.ref_station;
    c.t = t;
  }

  Json json(double t, double wall)
  {
    advance(t, wall);
    rotateRtcm(t);
    Json j;
    j["since"] = start_wall_;
    if (have_pvt_) {
      const auto & p = last_;
      j["now"] = {{"q", last_q_},           {"age", r1(t - pvt_t_)},     {"fix_type", p.fix_type},
                  {"sv", p.num_sv},         {"hacc", r3(p.h_acc_m)},     {"vacc", r3(p.v_acc_m)},
                  {"pdop", r2(p.p_dop)},    {"corr_age", p.corr_age_s},  {"diff", p.diff_soln},
                  {"speed", r2(p.g_speed)}};
    }
    Json sess = {{"seconds", {r1(seconds_[0]), r1(seconds_[1]), r1(seconds_[2]), r1(seconds_[3])}},
                 {"losses", losses_},
                 {"jumps", jumps_},
                 {"suspect_refix", suspect_refix_}};
    if (!refix_s_.empty()) {
      std::vector<double> v(refix_s_.begin(), refix_s_.end());
      std::sort(v.begin(), v.end());
      sess["refix"] = {{"last", r1(refix_s_.back())}, {"median", r1(v[v.size() / 2])}, {"max", r1(v.back())},
                       {"n", v.size()}};
    }
    if (loss_t_ >= 0) sess["lost_for"] = r1(t - loss_t_);
    j["session"] = sess;
    j["events"] = events_;
    j["sats"] = satsJson(t);
    if (rf_t_ >= 0 && t - rf_t_ < kFresh) {
      Json rf = Json::array();
      for (const auto & b : rf_) {
        rf.push_back({{"band", b.id == 0 ? "L1" : "L2/L5"}, {"jamming", b.jamming}, {"ant", b.ant_status},
                      {"power", b.ant_power}, {"noise", b.noise}, {"agc", b.agc}, {"jam_ind", b.jam_ind}});
      }
      j["rf"] = rf;
    }
    if (relpos_t_ >= 0 && t - relpos_t_ < kFresh) {
      j["base"] = {{"ref", relpos_.ref_station}, {"length", r2(relpos_.length_m)}, {"valid", relpos_.valid},
                   {"carr", relpos_.carr_soln},  {"pos_miss", relpos_.ref_pos_miss},
                   {"obs_miss", relpos_.ref_obs_miss}};
    }
    if (!rtcm_.empty()) {
      // [type, last minute, total, crc failed, used (0 ?, 1 no, 2 yes), age s, station, interval s (0 = one seen)]
      Json r = Json::array();
      for (const auto & [type, c] : rtcm_) {
        r.push_back({type, c.last_minute, c.total, c.crc, c.used, r1(t - c.t), c.ref, r1(c.interval)});
      }
      j["rtcm"] = r;
    }
    // [end wall, % fixed, % float, % no RTK, % no fix, top-4 C/N0, top-4 L1, top-4 L2, sats used]
    // (null = no data); the shares count only time with NAV-PVT, no data at all = all null
    j["hist"] = history_;
    return j;
  }

private:
  struct Bucket
  {
    std::array<double, 4> seconds{};
    double cno_sum = 0, used_sum = 0;
    int cno_n = 0, used_n = 0;
    std::array<double, 2> band_sum{};
    std::array<int, 2> band_n{};
  };

  struct RtcmCount
  {
    uint64_t total = 0, crc = 0;
    int minute = 0, last_minute = 0;
    uint8_t used = 0;
    uint16_t ref = 0;
    double t = 0;
    double interval = 0;  // smoothed gap between messages of this type
  };

  static double r1(double v) { return std::round(v * 10) / 10; }
  static double r2(double v) { return std::round(v * 100) / 100; }
  static double r3(double v) { return std::round(v * 1000) / 1000; }

  static double top4(std::vector<int> & v)
  {
    if (v.empty()) return 0;
    const size_t k = std::min<size_t>(4, v.size());
    std::partial_sort(v.begin(), v.begin() + static_cast<long>(k), v.end(), std::greater<int>());
    double s = 0;
    for (size_t i = 0; i < k; ++i) s += v[i];
    return s / static_cast<double>(k);
  }

  std::pair<double, double> local(const NavPvt & p) const
  {
    constexpr double kR = 6378137.0, kDeg = M_PI / 180.0;
    return {(p.lon_deg - lon0_) * kDeg * kR * std::cos(lat0_ * kDeg), (p.lat_deg - lat0_) * kDeg * kR};
  }

  void addEvent(Json e)
  {
    events_.push_back(std::move(e));
    if (events_.size() > 30) events_.pop_front();
  }

  void advance(double t, double wall)
  {
    if (bucket_t_ < 0) {
      bucket_t_ = t;
      start_wall_ = wall;
      return;
    }
    for (int i = 0; t - bucket_t_ >= kBucket && i < static_cast<int>(kHistory); ++i) {
      bucket_t_ += kBucket;
      closeBucket(wall - (t - bucket_t_));
    }
    if (t - bucket_t_ >= kBucket) bucket_t_ = t;  // long silence: skip ahead
  }

  void closeBucket(double end_wall)
  {
    const auto & b = bucket_;
    const double total = b.seconds[0] + b.seconds[1] + b.seconds[2] + b.seconds[3];
    auto pct = [&](int q) { return total > 0 ? Json(std::round(100 * b.seconds[q] / total)) : Json(); };
    auto avg = [](double s, int n) { return n > 0 ? Json(r1(s / n)) : Json(); };
    history_.push_back({std::round(end_wall), pct(3), pct(2), pct(1), pct(0), avg(b.cno_sum, b.cno_n),
                        avg(b.band_sum[0], b.band_n[0]), avg(b.band_sum[1], b.band_n[1]),
                        avg(b.used_sum, b.used_n)});
    if (history_.size() > kHistory) history_.pop_front();
    bucket_ = Bucket();
  }

  void rotateRtcm(double t)
  {
    if (rtcm_minute_t_ < 0) rtcm_minute_t_ = t;
    if (t - rtcm_minute_t_ < 60) return;
    const bool skipped = t - rtcm_minute_t_ >= 120;  // a whole minute without RTCM in between
    for (auto & [type, c] : rtcm_) {
      c.last_minute = skipped ? 0 : c.minute;
      c.minute = 0;
    }
    rtcm_minute_t_ = t;
  }

  // [gnss, sv, cno, elev, azim, used, diff corr, quality, L1 cno, L2 cno, carrier used]
  // (the last three from NAV-SIG, null without it)
  Json satsJson(double t) const
  {
    Json out = Json::array();
    if (sat_t_ < 0 || t - sat_t_ >= kFresh) return out;
    const bool sig = sig_t_ >= 0 && t - sig_t_ < kFresh;
    std::map<std::pair<int, int>, std::array<int, 3>> bands;  // L1 cno, L2 cno, carrier used
    if (sig) {
      for (const auto & s : sigs_) {
        auto & b = bands.try_emplace({s.gnss, s.sv}, std::array<int, 3>{0, 0, 0}).first->second;
        int & c = b[s.l1 ? 0 : 1];
        c = std::max<int>(c, s.cno);
        if (s.cr_used) b[2] = 1;
      }
    }
    for (const auto & s : sats_) {
      if (s.cno == 0 && !s.used) continue;
      Json row = {s.gnss, s.sv, s.cno, s.elev, s.azim, s.used ? 1 : 0, s.diff_corr ? 1 : 0, s.quality};
      if (const auto it = bands.find({s.gnss, s.sv}); it != bands.end()) {
        row.push_back(it->second[0]);
        row.push_back(it->second[1]);
        row.push_back(it->second[2]);
      } else {
        row.push_back(nullptr);
        row.push_back(nullptr);
        row.push_back(nullptr);
      }
      out.push_back(row);
    }
    return out;
  }

  // NAV-PVT
  bool have_pvt_ = false, have_origin_ = false;
  NavPvt last_;
  int last_q_ = 0;
  double pvt_t_ = 0;
  double lat0_ = 0, lon0_ = 0;
  double fix_e_ = 0, fix_n_ = 0;  // last RTK fixed position (local metres)
  std::array<double, 4> seconds_{};
  int losses_ = 0, jumps_ = 0, suspect_refix_ = 0;
  double loss_t_ = -1, loss_wall_ = 0, moved_ = 0;
  int worst_ = 3;
  std::deque<double> refix_s_;
  std::deque<Json> events_;
  // history
  double bucket_t_ = -1, start_wall_ = 0;
  Bucket bucket_;
  std::deque<Json> history_;
  // other messages
  std::vector<SatInfo> sats_;
  double sat_t_ = -1;
  std::vector<SigInfo> sigs_;
  double sig_t_ = -1;
  std::vector<RfBlock> rf_;
  double rf_t_ = -1;
  RelPosNed relpos_;
  double relpos_t_ = -1;
  std::map<uint16_t, RtcmCount> rtcm_;
  double rtcm_minute_t_ = -1;
};

}  // namespace open_mower_next::ubx_gps
