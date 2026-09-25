#pragma once
// Worx mainboard link protocol (Worx firmware by Daniel Wiegert).
//
// Full-duplex SPI, fixed 250-byte transfers. A message is SOF (0x01) + JSON (or
// a plain-text DEBUG/State line) + EOF (0xFF); unused bytes are NOP (0x00).
// Received messages may span transfers. The encoding mirrors the ROS1
// worx_comms node so the firmware needs no changes.
//
// ROS-free so it can be unit tested.

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace open_mower_next::worx_hardware
{

constexpr uint8_t kNop = 0x00;
constexpr uint8_t kSof = 0x01;
constexpr uint8_t kEof = 0xFF;
constexpr size_t kFrameLen = 250;

using Frame = std::array<uint8_t, kFrameLen>;

// SOF + payload + EOF, NOP padded. Returns false if the payload had to be
// truncated to fit (kFrameLen - 2 bytes).
bool encodeFrame(const std::string & payload, Frame & out);

// Stateful stream decoder: feed every received transfer, collect messages.
class FrameDecoder
{
public:
  void feed(const uint8_t * data, size_t len, std::vector<std::string> & out);
  size_t dropped() const { return dropped_; }

private:
  bool receiving_ = false;
  std::string buf_;
  size_t dropped_ = 0;
};

// ---- commands (host -> board) ----------------------------------------------
std::string cmdSetSpeed(int left, int right, int mow);  // {"MOTORREQ_SETSPEED":{"left":..,"right":..,"mow":..}}
std::string cmdMotorsEnable();                          // {"MOTORREQ_ENABLE":{}}
std::string cmdMotorsDisable();                         // {"MOTORREQ_DISABLE":{}}
std::string cmdPing(int count);                         // {"ping":{"count":..}} resets the firmware SPI watchdog

// ---- status (board -> host) ---------------------------------------------------
struct Battery
{
  int mv = 0;
  int ma = 0;
  int temp_raw = 0;  // 0.1 degC
  std::optional<int> cell_low, cell_high, in_charger;
};

// {"MotorPulse":{"Left":2068,"Right":2357,"Mow":182,"DirLeft":0,"DirRight":0,"Emergancy":0,"BlockForward":0}}
struct MotorPulse
{
  uint32_t left = 0, right = 0;  // cumulative tick magnitudes (count up in both directions)
  int mow = 0;                   // blade pulses per report (~180 while mowing), not cumulative
  bool dir_left = false, dir_right = false;  // true = reverse
  std::optional<int> emergency;      // firmware field "Emergancy"
  std::optional<int> block_forward;  // firmware field "BlockForward"
};

struct Triple
{
  int left = 0, right = 0, mow = 0;
};

struct BoardMessage
{
  bool is_json = false;
  std::string text;  // raw message
  std::optional<Battery> battery;
  std::optional<MotorPulse> motor_pulse;
  std::optional<Triple> motor_current;  // {"MotorCurrent":{"Left","Right","Mow"}} (older firmware: "MowRPM")
  std::optional<Triple> motor_pwm;      // {"MotorPWM":{"Left","Right","Mow"}}
  // {"Digital":{"Stuck":0,"Stuck2":0,"Door":1,"Door2":1,"Lift":1,"Collision":1,"Stop":0,"Rain":0}}
  // Raw. Door/Lift/Collision read 1 in the normal state (active low?); not interpreted yet.
  std::optional<std::map<std::string, int>> digital;
  std::optional<std::map<std::string, int>> analog;    // {"Rain":..,"boardTemp":..}
  std::optional<std::map<std::string, int>> boundary;  // perimeter wire signal
  std::optional<std::string> motor_state;  // MOTORREQ_ENABLE / _DISABLE / _SETSPEED
  std::optional<std::string> power_state;  // e.g. "StartCharging", "Charging"
};

// Never throws; non-JSON text (DEBUG..., State...) gives is_json = false.
BoardMessage parseMessage(const std::string & msg);

}  // namespace open_mower_next::worx_hardware
