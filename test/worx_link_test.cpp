// Worx link layer: framing, message parsing (real firmware lines), tick
// odometry, and the SPI loop against the fake board.
#include "worx_hardware/transport.hpp"
#include "worx_hardware/wheel_odometer.hpp"
#include "worx_hardware/worx_link.hpp"
#include "worx_hardware/worx_protocol.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <thread>

using namespace open_mower_next::worx_hardware;

TEST(WorxProtocol, EncodeFrameLayout)
{
  Frame f;
  EXPECT_TRUE(encodeFrame("{\"a\":1}", f));
  EXPECT_EQ(f[0], kSof);
  EXPECT_EQ(std::string(f.begin() + 1, f.begin() + 8), "{\"a\":1}");
  EXPECT_EQ(f[8], kEof);
  EXPECT_EQ(f[9], kNop);
  EXPECT_EQ(f[kFrameLen - 1], kNop);
}

TEST(WorxProtocol, EncodeTruncatesLongPayload)
{
  Frame f;
  EXPECT_FALSE(encodeFrame(std::string(400, 'x'), f));
  EXPECT_EQ(f[0], kSof);
  EXPECT_EQ(f[kFrameLen - 1], kEof);
}

TEST(WorxProtocol, CommandsMatchRos1KeyOrder)
{
  EXPECT_EQ(cmdSetSpeed(10, -20, 1850), R"({"MOTORREQ_SETSPEED":{"left":10,"right":-20,"mow":1850}})");
  EXPECT_EQ(cmdMotorsEnable(), R"({"MOTORREQ_ENABLE":{}})");
  EXPECT_EQ(cmdMotorsDisable(), R"({"MOTORREQ_DISABLE":{}})");
  EXPECT_EQ(cmdPing(7), R"({"ping":{"count":7}})");
}

TEST(WorxProtocol, DecoderHandlesSplitNopAndGarbage)
{
  FrameDecoder d;
  std::vector<std::string> out;
  const std::string a = "\x01{\"x\":";
  const std::string b = std::string("\x00\x00", 2) + "1}\xff junk \x01{\"y\":2}\xff";
  d.feed(reinterpret_cast<const uint8_t *>(a.data()), a.size(), out);
  EXPECT_TRUE(out.empty());
  d.feed(reinterpret_cast<const uint8_t *>(b.data()), b.size(), out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0], "{\"x\":1}");
  EXPECT_EQ(out[1], "{\"y\":2}");
  EXPECT_EQ(d.dropped(), 0u);
}

TEST(WorxProtocol, DecoderDropsMessageWithLostEof)
{
  FrameDecoder d;
  std::vector<std::string> out;
  const std::string s = "\x01{\"lost\":1\x01{\"ok\":1}\xff";
  d.feed(reinterpret_cast<const uint8_t *>(s.data()), s.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], "{\"ok\":1}");
  EXPECT_EQ(d.dropped(), 1u);

  std::string longmsg = "\x01" + std::string(300, 'a');
  d.feed(reinterpret_cast<const uint8_t *>(longmsg.data()), longmsg.size(), out);
  EXPECT_EQ(d.dropped(), 2u);
}

// Lines as logged by worx_comms on the robot (2026-09).
TEST(WorxProtocol, ParsesRealFirmwareMessages)
{
  auto p = parseMessage(
    R"({"MotorPulse":{"Left":2068,"Right":2357,"Mow":182,"DirLeft":0,"DirRight":1,"Emergancy":0,"BlockForward":1}})");
  ASSERT_TRUE(p.is_json);
  ASSERT_TRUE(p.motor_pulse);
  EXPECT_EQ(p.motor_pulse->left, 2068u);
  EXPECT_EQ(p.motor_pulse->right, 2357u);
  EXPECT_EQ(p.motor_pulse->mow, 182);
  EXPECT_FALSE(p.motor_pulse->dir_left);
  EXPECT_TRUE(p.motor_pulse->dir_right);
  EXPECT_EQ(p.motor_pulse->emergency, 0);
  EXPECT_EQ(p.motor_pulse->block_forward, 1);

  p = parseMessage(R"({"Battery":{"mV":28443,"mA":382,"Temp":236,"CellLow":1,"CellHigh":1,"InCharger":1}})");
  ASSERT_TRUE(p.battery);
  EXPECT_EQ(p.battery->mv, 28443);
  EXPECT_EQ(p.battery->ma, 382);
  EXPECT_EQ(p.battery->temp_raw, 236);
  EXPECT_EQ(p.battery->in_charger, 1);

  p = parseMessage(R"({"Digital":{"Stuck":0,"Stuck2":0,"Door":1,"Door2":1,"Lift":1,"Collision":1,"Stop":0,"Rain":0}})");
  ASSERT_TRUE(p.digital);
  EXPECT_EQ(p.digital->at("Lift"), 1);
  EXPECT_EQ(p.digital->size(), 8u);

  p = parseMessage(R"({"MotorCurrent":{"Left":102,"Right":15,"Mow":7}})");
  ASSERT_TRUE(p.motor_current);
  EXPECT_EQ(p.motor_current->mow, 7);
  p = parseMessage(R"({"MotorCurrent":{"Left":1,"Right":2,"MowRPM":9}})");
  EXPECT_EQ(p.motor_current->mow, 9);

  p = parseMessage(R"({"motorState":"MOTORREQ_SETSPEED"})");
  EXPECT_EQ(p.motor_state, "MOTORREQ_SETSPEED");
  p = parseMessage(R"({"powerState":"StartCharging"})");
  EXPECT_EQ(p.power_state, "StartCharging");

  p = parseMessage("DEBUG: Cal: 0 0 0, offset: 0 (Temp: 11, 247 )");
  EXPECT_FALSE(p.is_json);
  p = parseMessage("{broken");
  EXPECT_FALSE(p.is_json);
  p = parseMessage(R"({"MotorPulse":"nope","Battery":[1,2]})");
  EXPECT_TRUE(p.is_json);
  EXPECT_FALSE(p.motor_pulse);
  EXPECT_FALSE(p.battery);
}

TEST(WheelOdometer, MagnitudeCounterWithDirectionBit)
{
  WheelOdometer o(414.0);
  EXPECT_DOUBLE_EQ(o.update(1000, false), 0.0);  // baseline
  EXPECT_NEAR(o.update(1414, false), 1.0, 1e-9);
  EXPECT_NEAR(o.update(1621, true), -0.5, 1e-9);  // counter still counts up while reversing
  EXPECT_NEAR(o.distance(), 0.5, 1e-9);
}

TEST(WheelOdometer, WrapAndGlitch)
{
  WheelOdometer o(100.0, 16, 2000);
  o.update(65500, false);
  EXPECT_NEAR(o.update(64, false), 1.0, 1e-9);  // 65500 -> 65535 -> 64 = 100 ticks
  EXPECT_DOUBLE_EQ(o.update(40000, false), 0.0);  // counter reset/garbage
  EXPECT_EQ(o.glitches(), 1u);
  EXPECT_NEAR(o.update(40100, false), 1.0, 1e-9);  // re-baselined
}

TEST(WorxLink, TalksToFakeBoard)
{
  FakeBoardTransport::Options fo;
  auto fake = std::make_unique<FakeBoardTransport>(fo);
  FakeBoardTransport * board = fake.get();
  WorxLink::Options lo;
  lo.ping_period = std::chrono::milliseconds(50);
  WorxLink link(std::move(fake), lo);

  std::mutex m;
  std::vector<std::string> rx;
  std::string error;
  ASSERT_TRUE(link.start([&](const std::string & s) { std::lock_guard<std::mutex> l(m); rx.push_back(s); }, error)) << error;

  link.send(cmdMotorsEnable());
  link.send(cmdSetSpeed(615, 615, 1850));  // 0.5 m/s
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  EXPECT_TRUE(board->enabled());
  EXPECT_EQ(board->leftPwm(), 615);
  EXPECT_EQ(board->mowPwm(), 1850);
  EXPECT_GE(board->pings(), 5);
  EXPECT_LT(link.secondsSinceRx(), 0.2);

  // Integrate the reported ticks like the hardware interface does.
  WheelOdometer odo(fo.ticks_per_m);
  bool state_seen = false;
  {
    std::lock_guard<std::mutex> l(m);
    for (const auto & s : rx) {
      auto p = parseMessage(s);
      if (p.motor_pulse) odo.update(p.motor_pulse->left, p.motor_pulse->dir_left);
      if (p.motor_state && *p.motor_state == "MOTORREQ_ENABLE") state_seen = true;
    }
  }
  EXPECT_TRUE(state_seen);
  EXPECT_NEAR(odo.distance(), 0.5 * 0.6, 0.12);  // ~0.3 m in 0.6 s

  link.send(cmdSetSpeed(-615, -615, 0));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  {
    std::lock_guard<std::mutex> l(m);
    WheelOdometer o2(fo.ticks_per_m);
    double before = 0;
    for (const auto & s : rx) {
      auto p = parseMessage(s);
      if (p.motor_pulse) o2.update(p.motor_pulse->left, p.motor_pulse->dir_left);
    }
    before = o2.distance();
    EXPECT_LT(before, odo.distance());  // reversed
  }
  link.stop();
}
