#include "ubx_gps/elevation_grid.hpp"
#include "ubx_gps/gps_stats.hpp"
#include "ubx_gps/ubx.hpp"

#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

using namespace open_mower_next::ubx_gps;

namespace
{
std::vector<uint8_t> frame(uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> f{0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size()),
                         static_cast<uint8_t>(payload.size() >> 8)};
  f.insert(f.end(), payload.begin(), payload.end());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size(); ++i) {
    a += f[i];
    b += a;
  }
  f.push_back(a);
  f.push_back(b);
  return f;
}

void put32(std::vector<uint8_t> & p, size_t at, int32_t v)
{
  for (int i = 0; i < 4; ++i) p[at + i] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> (8 * i));
}

std::vector<uint8_t> pvtPayload()
{
  std::vector<uint8_t> p(92, 0);
  p[20] = 3;                 // 3D
  p[21] = 0x01 | (2 << 6);   // gnssFixOK, RTK fixed
  p[23] = 29;
  put32(p, 24, 124685912);   // lon
  put32(p, 28, 571746646);   // lat
  put32(p, 32, 81234);       // height mm
  put32(p, 36, 49876);       // hMSL mm
  put32(p, 40, 14);          // hAcc mm
  put32(p, 44, 20);          // vAcc mm
  return p;
}
}  // namespace

TEST(Ubx, DecodesNavPvt)
{
  const auto n = parseNavPvt(pvtPayload().data(), 92);
  ASSERT_TRUE(n);
  EXPECT_TRUE(n->fix_ok);
  EXPECT_EQ(n->carr_soln, 2);
  EXPECT_EQ(n->num_sv, 29);
  EXPECT_NEAR(n->lat_deg, 57.1746646, 1e-9);
  EXPECT_NEAR(n->lon_deg, 12.4685912, 1e-9);
  EXPECT_NEAR(n->height_m, 81.234, 1e-9);
  EXPECT_NEAR(n->h_msl_m, 49.876, 1e-9);
  EXPECT_NEAR(n->h_acc_m, 0.014, 1e-9);
  EXPECT_FALSE(parseNavPvt(pvtPayload().data(), 91));
}

TEST(Ubx, SplitsStreamAndSkipsGarbage)
{
  std::vector<uint8_t> stream{'$', 'G', 'N', 0xB5, 0x00};
  const auto f1 = frame(0x01, 0x07, pvtPayload());
  auto bad = frame(0x01, 0x07, pvtPayload());
  bad[20] ^= 0xFF;  // checksum no longer matches
  const auto f2 = frame(0x01, 0x35, {1, 2, 3});
  stream.insert(stream.end(), f1.begin(), f1.end());
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), f2.begin(), f2.end());

  std::vector<std::pair<int, size_t>> got;
  FrameParser parser([&](uint8_t, uint8_t id, const uint8_t *, size_t len) { got.emplace_back(id, len); });
  // Byte by byte: frames split across reads.
  for (uint8_t b : stream) parser.feed(&b, 1);
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(got[0], (std::pair<int, size_t>{0x07, 92}));
  EXPECT_EQ(got[1], (std::pair<int, size_t>{0x35, 3}));
}

TEST(Ubx, DecodesNavPvtQualityFields)
{
  auto p = pvtPayload();
  p[21] |= 0x02;               // diffSoln
  put32(p, 60, 312);           // gSpeed mm/s
  p[76] = 0x87;                // pDOP 1.35
  p[77] = 0x00;
  p[78] = 3 << 1;              // lastCorrectionAge: 2..5 s
  const auto n = parseNavPvt(p.data(), 92);
  ASSERT_TRUE(n);
  EXPECT_TRUE(n->diff_soln);
  EXPECT_NEAR(n->g_speed, 0.312, 1e-9);
  EXPECT_NEAR(n->p_dop, 1.35, 1e-9);
  EXPECT_EQ(n->corr_age_s, 5);
  p[78] = 0;
  EXPECT_EQ(parseNavPvt(p.data(), 92)->corr_age_s, -1);
}

TEST(Ubx, DecodesNavSatAndSig)
{
  std::vector<uint8_t> sat(8 + 24, 0);
  sat[4] = 1;
  sat[5] = 2;
  const std::vector<uint8_t> s0{0, 12, 44, 63, 0x2c, 0x01, 0, 0, 0x4f, 0, 0, 0};  // GPS 12, az 300, used, diffCorr
  const std::vector<uint8_t> s1{6, 3, 21, static_cast<uint8_t>(-5), 10, 0, 0, 0, 0x01, 0, 0, 0};
  std::copy(s0.begin(), s0.end(), sat.begin() + 8);
  std::copy(s1.begin(), s1.end(), sat.begin() + 20);
  const auto v = parseNavSat(sat.data(), sat.size());
  ASSERT_TRUE(v);
  ASSERT_EQ(v->size(), 2u);
  EXPECT_EQ((*v)[0].sv, 12);
  EXPECT_EQ((*v)[0].cno, 44);
  EXPECT_EQ((*v)[0].elev, 63);
  EXPECT_EQ((*v)[0].azim, 300);
  EXPECT_EQ((*v)[0].quality, 7);
  EXPECT_TRUE((*v)[0].used);
  EXPECT_TRUE((*v)[0].diff_corr);
  EXPECT_EQ((*v)[1].gnss, 6);
  EXPECT_EQ((*v)[1].elev, -5);
  EXPECT_FALSE((*v)[1].used);
  EXPECT_FALSE(parseNavSat(sat.data(), sat.size() - 1));

  std::vector<uint8_t> sig(8 + 32, 0);
  sig[5] = 2;
  const std::vector<uint8_t> g0{0, 12, 0, 0, 0, 0, 45, 7, 0, 0, 0x18, 0, 0, 0, 0, 0};  // L1C/A, pr+cr used
  const std::vector<uint8_t> g1{2, 5, 5, 0, 0, 0, 38, 4, 0, 0, 0x08, 0, 0, 0, 0, 0};   // E5bI, pr used
  std::copy(g0.begin(), g0.end(), sig.begin() + 8);
  std::copy(g1.begin(), g1.end(), sig.begin() + 24);
  const auto w = parseNavSig(sig.data(), sig.size());
  ASSERT_TRUE(w);
  ASSERT_EQ(w->size(), 2u);
  EXPECT_TRUE((*w)[0].l1);
  EXPECT_TRUE((*w)[0].cr_used);
  EXPECT_FALSE((*w)[1].l1);
  EXPECT_EQ((*w)[1].cno, 38);
  EXPECT_TRUE((*w)[1].pr_used);
  EXPECT_FALSE((*w)[1].cr_used);
  EXPECT_FALSE(isL1Signal(0, 3));  // GPS L2CL
  EXPECT_TRUE(isL1Signal(3, 1));   // BeiDou B1I D2
  EXPECT_FALSE(isL1Signal(6, 2));  // GLONASS L2OF
}

TEST(Ubx, DecodesMonRfRtcmRelPos)
{
  std::vector<uint8_t> rf(4 + 48, 0);
  rf[1] = 2;
  rf[4 + 0] = 0;
  rf[4 + 1] = 2;   // jamming warning
  rf[4 + 2] = 4;   // antenna open
  rf[4 + 3] = 1;
  rf[4 + 12] = 0x58; rf[4 + 13] = 0x00;  // noise 88
  rf[4 + 14] = 0x10; rf[4 + 15] = 0x0a;  // agc 2576
  rf[4 + 16] = 17;
  rf[28] = 1;
  const auto r = parseMonRf(rf.data(), rf.size());
  ASSERT_TRUE(r);
  ASSERT_EQ(r->size(), 2u);
  EXPECT_EQ((*r)[0].jamming, 2);
  EXPECT_EQ((*r)[0].ant_status, 4);
  EXPECT_EQ((*r)[0].noise, 88);
  EXPECT_EQ((*r)[0].agc, 2576);
  EXPECT_EQ((*r)[0].jam_ind, 17);
  EXPECT_EQ((*r)[1].id, 1);

  const std::vector<uint8_t> rtcm{2, 0x05, 0, 0, 0x2a, 0, 0x3d, 0x04};  // crc failed, used, station 42, 1085
  const auto m = parseRxmRtcm(rtcm.data(), rtcm.size());
  ASSERT_TRUE(m);
  EXPECT_EQ(m->type, 1085);
  EXPECT_EQ(m->ref_station, 42);
  EXPECT_TRUE(m->crc_failed);
  EXPECT_EQ(m->used, 2);

  std::vector<uint8_t> rel(64, 0);
  rel[0] = 1;
  rel[2] = 7;
  put32(rel, 20, 412345);  // 4123.45 m
  rel[35] = 50;            // + 5 mm
  put32(rel, 60, 0x04 | (2 << 3) | 0x80);
  const auto b = parseNavRelPosNed(rel.data(), rel.size());
  ASSERT_TRUE(b);
  EXPECT_EQ(b->ref_station, 7);
  EXPECT_NEAR(b->length_m, 4123.455, 1e-6);
  EXPECT_TRUE(b->valid);
  EXPECT_EQ(b->carr_soln, 2);
  EXPECT_TRUE(b->ref_obs_miss);
  EXPECT_FALSE(b->ref_pos_miss);
}

namespace
{
NavPvt pvt(int carr, double north_m, double speed = 0.0)
{
  NavPvt p;
  p.fix_ok = true;
  p.fix_type = 3;
  p.carr_soln = static_cast<uint8_t>(carr);
  p.num_sv = 30;
  p.lat_deg = 57.0 + north_m / 111319.49;  // R * pi / 180 m per degree
  p.lon_deg = 12.0;
  p.g_speed = speed;
  return p;
}
}  // namespace

TEST(GpsStats, CountsLossesRefixOffsetsAndJumps)
{
  GpsStats s;
  double t = 0;
  for (int i = 0; i < 8; ++i, t += 0.25) s.onPvt(pvt(1, 0.0), t, 1000 + t);   // float at the start: no loss
  for (int i = 0; i < 8; ++i, t += 0.25) s.onPvt(pvt(2, 0.0), t, 1000 + t);   // fixed at 0
  for (int i = 0; i < 12; ++i, t += 0.25) s.onPvt(pvt(1, 0.5), t, 1000 + t);          // float 3 s, standing
  s.onPvt(pvt(2, 2.0), t, 1000 + t);                                                  // fixed again 2 m off
  t += 0.25;
  s.onPvt(pvt(2, 2.05), t, 1000 + t);  // normal step
  t += 0.25;
  s.onPvt(pvt(2, 2.6), t, 1000 + t);   // 0.55 m in 0.25 s standing: a jump
  const auto j = s.json(t, 1000 + t);
  const auto & sess = j["session"];
  EXPECT_EQ(sess["losses"], 1);
  EXPECT_EQ(sess["jumps"], 1);
  EXPECT_EQ(sess["suspect_refix"], 1);
  const auto & ev = j["events"];
  ASSERT_GE(ev.size(), 2u);
  const auto & refix = ev[ev.size() - 2];
  EXPECT_EQ(refix["kind"], "refix");
  EXPECT_NEAR(refix["offset"].get<double>(), 2.0, 0.01);
  EXPECT_NEAR(refix["lost"].get<double>(), 3.0, 0.01);
  EXPECT_EQ(refix["worst"], 2);
  EXPECT_TRUE(refix["suspect"].get<bool>());
  EXPECT_EQ(ev.back()["kind"], "jump");
  EXPECT_EQ(j["now"]["q"], 3);
}

TEST(GpsStats, RefixAfterDrivingIsNotSuspect)
{
  GpsStats s;
  double t = 0;
  for (int i = 0; i < 4; ++i, t += 0.25) s.onPvt(pvt(2, 0.0), t, t);
  for (int i = 0; i < 8; ++i, t += 0.25) s.onPvt(pvt(1, 0.3 * i * 0.25, 0.3), t, t);  // drove 0.6 m
  s.onPvt(pvt(2, 0.62, 0.3), t, t);
  const auto j = s.json(t, t);
  EXPECT_EQ(j["session"]["suspect_refix"], 0);
  EXPECT_FALSE(j["events"].back()["suspect"].get<bool>());
}

TEST(GpsStats, HistoryBucketsAndSatellites)
{
  GpsStats s;
  double t = 0;
  std::vector<SatInfo> sats;
  for (uint8_t i = 1; i <= 6; ++i) sats.push_back({0, i, static_cast<uint8_t>(30 + i), 40, 90, 7, i > 2, true});
  sats.push_back({6, 9, 0, 10, 10, 0, false, false});  // not tracked: left out
  for (; t < 15; t += 0.25) {
    s.onPvt(pvt(2, 0.0), t, 5000 + t);
    s.onSat(sats, t);
  }
  for (; t < 20.1; t += 0.25) s.onPvt(pvt(1, 0.0), t, 5000 + t);
  s.onPvt(pvt(1, 0.0), 45, 5045);  // receiver silent 25 s
  const auto j = s.json(45, 5045);
  ASSERT_EQ(j["hist"].size(), 2u);
  const auto & h = j["hist"][0];
  EXPECT_EQ(h[0], 5020);
  EXPECT_EQ(h[1], 76);           // % fixed: 15 of 19.75 s (the last sample's time counts in the next one)
  EXPECT_EQ(h[2], 24);           // % float
  EXPECT_NEAR(h[5].get<double>(), 34.5, 1e-9);  // top 4 of 31..36
  EXPECT_TRUE(h[6].is_null());   // no NAV-SIG
  EXPECT_EQ(h[8], 30);
  EXPECT_EQ(j["hist"][1][2], 100);   // only the 0.25 s before the silence
  EXPECT_TRUE(j["hist"][1][5].is_null());
  EXPECT_EQ(j["sats"].size(), 0u);  // NAV-SAT older than 5 s
  s.onSat(sats, 45);
  const auto k = s.json(45.5, 5045.5);
  ASSERT_EQ(k["sats"].size(), 6u);
  EXPECT_EQ(k["sats"][0][2], 31);
  EXPECT_TRUE(k["sats"][0][8].is_null());
}

TEST(GpsStats, RtcmPerMinute)
{
  GpsStats s;
  for (int i = 0; i < 60; ++i) s.onRtcm({1077, 3, false, 2}, i);
  s.onRtcm({1005, 3, true, 1}, 30);
  const auto j = s.json(61, 61);
  ASSERT_EQ(j["rtcm"].size(), 2u);
  EXPECT_EQ(j["rtcm"][0][0], 1005);
  EXPECT_EQ(j["rtcm"][0][3], 1);  // crc failed
  EXPECT_EQ(j["rtcm"][1][1], 60);
  EXPECT_EQ(j["rtcm"][1][4], 2);
  EXPECT_DOUBLE_EQ(j["rtcm"][0][7].get<double>(), 0.0);  // only one seen
  EXPECT_DOUBLE_EQ(j["rtcm"][1][7].get<double>(), 1.0);
}

TEST(GpsStats, RtcmIntervalIgnoresOutage)
{
  GpsStats s;
  for (int i = 0; i < 5; ++i) s.onRtcm({1005, 3, false, 2}, 30.0 * i);
  s.onRtcm({1005, 3, false, 2}, 120 + 300);  // 5 min outage counts as 60 s
  const auto j = s.json(421, 421);
  EXPECT_NEAR(j["rtcm"][0][7].get<double>(), 39.0, 0.05);
}

TEST(ElevationGrid, AveragesPerCell)
{
  ElevationGrid g(0.2);
  g.add(0.05, 0.05, 10.0);
  g.add(0.15, 0.19, 10.2);
  g.add(-0.05, 0.05, 5.0);  // other side of 0: its own cell
  EXPECT_EQ(g.size(), 2u);
  EXPECT_NEAR(g.mean(0.1, 0.1), 10.1, 1e-12);
  EXPECT_EQ(g.count(0.1, 0.1), 2u);
  EXPECT_NEAR(g.mean(-0.1, 0.1), 5.0, 1e-12);
  EXPECT_TRUE(std::isnan(g.mean(1.0, 1.0)));
}

TEST(ElevationGrid, FullCellTakesNoMore)
{
  const std::string path = ::testing::TempDir() + "elevation_full_test.csv";
  ElevationGrid g(0.2, 2);
  EXPECT_TRUE(g.add(0.1, 0.1, 1.0));
  EXPECT_TRUE(g.add(0.1, 0.1, 2.0));
  ASSERT_TRUE(g.save(path, "", 1));
  EXPECT_FALSE(g.add(0.1, 0.1, 9.0));
  EXPECT_FALSE(g.dirty());
  EXPECT_NEAR(g.mean(0.1, 0.1), 1.5, 1e-12);
  std::remove(path.c_str());
}

TEST(ElevationGrid, SavesAndMergesOnLoad)
{
  const std::string path = ::testing::TempDir() + "elevation_grid_test.csv";
  ElevationGrid a(0.2);
  for (double z : {1.0, 1.1, 1.2}) a.add(0.3, -0.3, z);
  a.add(2.0, 2.0, 7.0);  // one sample: left out with min_samples 2
  ASSERT_TRUE(a.save(path, "# test\n", 2));
  EXPECT_FALSE(a.dirty());

  std::ifstream f(path);
  std::string l1, l2, l3, l4;
  std::getline(f, l1);
  std::getline(f, l2);
  std::getline(f, l3);
  EXPECT_EQ(l1, "# test");
  EXPECT_EQ(l2, "x,y,z,n,std");
  EXPECT_EQ(l3, "0.30,-0.30,1.1000,3,0.1000");
  EXPECT_FALSE(std::getline(f, l4));

  // Loading into a grid that already has samples equals adding all of them.
  ElevationGrid b(0.2);
  b.add(0.25, -0.25, 1.3);
  EXPECT_EQ(b.load(path), 1);
  EXPECT_EQ(b.count(0.3, -0.3), 4u);
  EXPECT_NEAR(b.mean(0.3, -0.3), 1.15, 1e-9);
  ElevationGrid c(0.2);
  for (double z : {1.0, 1.1, 1.2, 1.3}) c.add(0.3, -0.3, z);
  auto rows = [&](ElevationGrid & g) {
    EXPECT_TRUE(g.save(path, "", 1));
    std::ifstream in(path);
    std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return all;
  };
  EXPECT_EQ(rows(b), rows(c));
  EXPECT_EQ(b.load("/nonexistent/elevation.csv"), -1);
  std::remove(path.c_str());
}
