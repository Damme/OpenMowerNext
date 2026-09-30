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
// Longest message the decoder keeps (firmware: 250 bytes of JSON + "#hhhh").
constexpr size_t kMaxMessage = 512;

using Frame = std::array<uint8_t, kFrameLen>;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) - same as the firmware.
uint16_t crc16(const std::string & data);
// payload + "#hhhh" (uppercase hex CRC of payload). Firmware 2026-09-30+ checks it;
// older firmware parses the JSON and ignores the suffix.
std::string withCrc(const std::string & payload);
// Checks a received message for a "#hhhh" suffix and strips it.
enum class CrcCheck { kNone, kOk, kBad };
CrcCheck checkAndStripCrc(std::string & msg);

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
std::string cmdResetEmergency();                        // {"MOTORREQ_RESETEMG":{}} (firmware 2026-09+)

// ---- status (board -> host) ---------------------------------------------------
struct Battery
{
  int mv = 0;
  int ma = 0;
  int temp_raw = 0;  // 0.1 degC
  std::optional<int> cell_low, cell_high, in_charger;
  // Firmware 2026-09+: powerState, CHARGER_CONNECTED and CHARGER_ENABLE pins
  std::optional<std::string> state;
  std::optional<int> contact, charge_enable;
};

// Firmware emergency reasons (MotorPulse "EmgReason" bits)
constexpr int kEmgTilt = 1 << 0;
constexpr int kEmgStop = 1 << 1;
constexpr int kEmgLift = 1 << 2;
std::string emergencyReasonText(int reason);  // "tilt+stop", "" for 0

// {"MotorPulse":{"Left":2068,"Right":2357,"Mow":182,"DirLeft":0,"DirRight":0,"Emergancy":0,"BlockForward":0}}
// Firmware 2026-09+ adds "EmgReason", "Motors", "Bumps" and "ms" (see the fields).
struct MotorPulse
{
  uint32_t left = 0, right = 0;  // cumulative tick magnitudes (count up in both directions)
  int mow = 0;                   // blade pulses per report (~180 while mowing), not cumulative
  bool dir_left = false, dir_right = false;  // true = reverse
  std::optional<int> emergency;      // firmware field "Emergancy"
  std::optional<int> block_forward;  // firmware field "BlockForward"
  std::optional<int> emergency_reason;  // kEmg* bits, latched with Emergancy
  std::optional<int> motors;            // real motor enable state
  std::optional<uint32_t> bumps;        // debounced bumper presses since boot
  std::optional<uint32_t> ms;           // board time of the sample [ms]
  std::optional<int> blade_lock;        // firmware cut the blade on Lift, until mow 0 is sent
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
  // A Digital frame without exactly the eight keys above was garbled on the wire
  // (real robot 2026-09-28, at charger contact: {"Stuck":0,...,"Lift":0,"Coll":0},
  // a false Lift): digital stays empty, the raw frame is kept here.
  std::optional<std::string> digital_corrupt;
  std::optional<std::map<std::string, int>> analog;    // {"Rain":..,"boardTemp":..}
  std::optional<std::map<std::string, int>> boundary;  // perimeter wire signal
  // Firmware 2026-09-30+, every 5 s: RxFrames, RxCrcErr, RxNoCrc, RxOverflow, TxFrames, TxDropped, DmaErr
  std::optional<std::map<std::string, int>> link;
  // Old firmware: last request (mostly _SETSPEED). 2026-09+: real state, one of
  // MOTORREQ_ENABLE / _DISABLE / _IDLE / _EMGSTOP.
  std::optional<std::string> motor_state;
  std::optional<std::string> power_state;  // e.g. "StartCharging", "Charging"
};

// Never throws; non-JSON text (DEBUG..., State...) gives is_json = false.
BoardMessage parseMessage(const std::string & msg);

}  // namespace open_mower_next::worx_hardware
