#include "worx_hardware/worx_protocol.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstring>

namespace open_mower_next::worx_hardware
{

// ordered_json keeps the key order of the ROS1 node (the firmware parser may care).
using ojson = nlohmann::ordered_json;

bool encodeFrame(const std::string & payload, Frame & out)
{
  out.fill(kNop);
  const size_t len = std::min(payload.size(), kFrameLen - 2);
  out[0] = kSof;
  std::memcpy(out.data() + 1, payload.data(), len);
  out[1 + len] = kEof;
  return len == payload.size();
}

void FrameDecoder::feed(const uint8_t * data, size_t len, std::vector<std::string> & out)
{
  for (size_t i = 0; i < len; ++i) {
    const uint8_t b = data[i];
    if (b == kSof) {
      // JSON never contains 0x01: a SOF while receiving means the EOF was lost.
      if (receiving_ && !buf_.empty()) {
        ++dropped_;
      }
      receiving_ = true;
      buf_.clear();
      continue;
    }
    if (!receiving_) {
      continue;
    }
    if (b == kEof) {
      if (!buf_.empty()) {
        out.push_back(buf_);
      }
      receiving_ = false;
      buf_.clear();
    } else if (b != kNop) {
      if (buf_.size() >= kFrameLen - 1) {
        ++dropped_;  // no EOF within a frame's worth of bytes
        receiving_ = false;
        buf_.clear();
        continue;
      }
      buf_.push_back(static_cast<char>(b));
    }
  }
}

std::string cmdSetSpeed(int left, int right, int mow)
{
  ojson j;
  j["MOTORREQ_SETSPEED"] = {{"left", left}, {"right", right}, {"mow", mow}};
  return j.dump();
}

std::string cmdMotorsEnable()
{
  ojson j;
  j["MOTORREQ_ENABLE"] = ojson::object();
  return j.dump();
}

std::string cmdMotorsDisable()
{
  ojson j;
  j["MOTORREQ_DISABLE"] = ojson::object();
  return j.dump();
}

std::string cmdPing(int count)
{
  ojson j;
  j["ping"] = {{"count", count}};
  return j.dump();
}

namespace
{
std::optional<long long> intField(const nlohmann::json & obj, const char * key)
{
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_number()) {
    return std::nullopt;
  }
  return it->is_number_float() ? static_cast<long long>(it->get<double>()) : it->get<long long>();
}

std::map<std::string, int> intMap(const nlohmann::json & obj)
{
  std::map<std::string, int> out;
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    if (it.value().is_number()) {
      out[it.key()] = static_cast<int>(it.value().get<double>());
    }
  }
  return out;
}
}  // namespace

BoardMessage parseMessage(const std::string & msg)
{
  BoardMessage m;
  m.text = msg;
  if (msg.empty() || msg.front() != '{') {
    return m;
  }
  const auto j = nlohmann::json::parse(msg, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    return m;
  }
  m.is_json = true;

  if (auto it = j.find("Battery"); it != j.end() && it->is_object()) {
    Battery b;
    b.mv = static_cast<int>(intField(*it, "mV").value_or(0));
    b.ma = static_cast<int>(intField(*it, "mA").value_or(0));
    b.temp_raw = static_cast<int>(intField(*it, "Temp").value_or(0));
    if (auto v = intField(*it, "CellLow")) b.cell_low = static_cast<int>(*v);
    if (auto v = intField(*it, "CellHigh")) b.cell_high = static_cast<int>(*v);
    if (auto v = intField(*it, "InCharger")) b.in_charger = static_cast<int>(*v);
    m.battery = b;
  }
  if (auto it = j.find("MotorPulse"); it != j.end() && it->is_object()) {
    MotorPulse p;
    p.left = static_cast<uint32_t>(intField(*it, "Left").value_or(0));
    p.right = static_cast<uint32_t>(intField(*it, "Right").value_or(0));
    p.mow = static_cast<int>(intField(*it, "Mow").value_or(0));
    p.dir_left = intField(*it, "DirLeft").value_or(0) != 0;
    p.dir_right = intField(*it, "DirRight").value_or(0) != 0;
    if (auto v = intField(*it, "Emergancy")) p.emergency = static_cast<int>(*v);
    if (auto v = intField(*it, "BlockForward")) p.block_forward = static_cast<int>(*v);
    m.motor_pulse = p;
  }
  if (auto it = j.find("MotorCurrent"); it != j.end() && it->is_object()) {
    m.motor_current = Triple{
      static_cast<int>(intField(*it, "Left").value_or(0)),
      static_cast<int>(intField(*it, "Right").value_or(0)),
      static_cast<int>(intField(*it, "Mow").value_or(intField(*it, "MowRPM").value_or(0)))};
  }
  if (auto it = j.find("MotorPWM"); it != j.end() && it->is_object()) {
    m.motor_pwm = Triple{
      static_cast<int>(intField(*it, "Left").value_or(0)),
      static_cast<int>(intField(*it, "Right").value_or(0)),
      static_cast<int>(intField(*it, "Mow").value_or(0))};
  }
  if (auto it = j.find("Digital"); it != j.end() && it->is_object()) {
    m.digital = intMap(*it);
  }
  if (auto it = j.find("Analog"); it != j.end() && it->is_object()) {
    m.analog = intMap(*it);
  }
  if (auto it = j.find("Boundary"); it != j.end() && it->is_object()) {
    m.boundary = intMap(*it);
  }
  if (auto it = j.find("motorState"); it != j.end() && it->is_string()) {
    m.motor_state = it->get<std::string>();
  }
  if (auto it = j.find("powerState"); it != j.end()) {
    m.power_state = it->is_string() ? it->get<std::string>() : it->dump();
  }
  return m;
}

}  // namespace open_mower_next::worx_hardware
