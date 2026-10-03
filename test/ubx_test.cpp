#include "ubx_gps/elevation_grid.hpp"
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
