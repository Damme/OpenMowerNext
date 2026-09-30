// INA226 driver against a fake chip: ID check, configuration, scaling and sign.
#include "ina226/ina226.hpp"

#include <gtest/gtest.h>

#include <map>

using namespace open_mower_next::ina226;

namespace
{
struct FakeChip : RegisterBus
{
  std::map<uint8_t, uint16_t> regs{{0x00, 0x4127}, {0xFE, 0x5449}, {0xFF, 0x2260}};
  bool fail = false;
  bool read16(uint8_t reg, uint16_t & value) override
  {
    if (fail) return false;
    value = regs[reg];
    return true;
  }
  bool write16(uint8_t reg, uint16_t value) override
  {
    if (fail) return false;
    regs[reg] = value;
    return true;
  }
};
}  // namespace

TEST(Ina226, ConfigWord)
{
  EXPECT_EQ(Ina226::configWord(1, 1100), 0x4127);   // power-on default
  EXPECT_EQ(Ina226::configWord(64, 1100), 0x4727);
  EXPECT_EQ(Ina226::configWord(100, 600), 0x46DF);  // rounded down: 64, 588 us
  EXPECT_EQ(Ina226::configWord(4096, 99999), 0x4FFF);
}

TEST(Ina226, InitWritesConfig)
{
  auto chip = std::make_unique<FakeChip>();
  auto * c = chip.get();
  Ina226 ina(std::move(chip), 0.02);
  std::string error;
  ASSERT_TRUE(ina.init(0x4727, error)) << error;
  EXPECT_EQ(c->regs[0x00], 0x4727);
}

TEST(Ina226, RejectsOtherChip)
{
  auto chip = std::make_unique<FakeChip>();
  chip->regs[0xFF] = 0x2270;
  Ina226 ina(std::move(chip), 0.02);
  std::string error;
  EXPECT_FALSE(ina.init(0x4727, error));
  EXPECT_NE(error.find("not an INA226"), std::string::npos);
}

TEST(Ina226, ScalesReadings)
{
  auto chip = std::make_unique<FakeChip>();
  // Values read on MrChoppie while charging (2026-09-30).
  chip->regs[0x01] = 0xF034;  // -4044 * 2.5 uV = -10.11 mV
  chip->regs[0x02] = 0x5A08;  // 23048 * 1.25 mV = 28.81 V
  auto * c = chip.get();
  Ina226 ina(std::move(chip), 0.02);
  Reading r;
  ASSERT_TRUE(ina.read(r));
  EXPECT_NEAR(r.bus_voltage, 28.81, 1e-3);
  EXPECT_NEAR(r.shunt_voltage, -0.01011, 1e-6);
  EXPECT_NEAR(r.current, -0.5055, 1e-4);
  EXPECT_NEAR(r.power, 28.81 * -0.5055, 1e-3);
  c->fail = true;
  EXPECT_FALSE(ina.read(r));
}
