#include "ubx_gps/ubx.hpp"

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
