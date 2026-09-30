#pragma once
// TI INA226 current/voltage monitor (datasheet SBOS547A). ROS-free; the
// register access is abstract so the driver can be tested against a fake chip.
//
// Current is computed from the shunt voltage register and shunt_ohms, not via
// the chip's calibration/current registers, so nothing depends on what another
// program wrote into CALIBRATION.

#include <cstdint>
#include <memory>
#include <string>

namespace open_mower_next::ina226
{

class RegisterBus
{
public:
  virtual ~RegisterBus() = default;
  virtual bool read16(uint8_t reg, uint16_t & value) = 0;  // big-endian on the wire
  virtual bool write16(uint8_t reg, uint16_t value) = 0;
};

// /dev/i2c-N via i2c-dev.
std::unique_ptr<RegisterBus> openLinuxI2c(const std::string & device, uint8_t address, std::string & error);

struct Reading
{
  double bus_voltage = 0.0;    // V (VBUS pin to GND)
  double shunt_voltage = 0.0;  // V, signed (IN+ minus IN-)
  double current = 0.0;        // A, signed as the chip sees it: shunt_voltage / shunt_ohms
  double power = 0.0;          // W, bus_voltage * current
};

class Ina226
{
public:
  static constexpr uint8_t kRegConfig = 0x00;
  static constexpr uint8_t kRegShunt = 0x01;
  static constexpr uint8_t kRegBus = 0x02;
  static constexpr uint8_t kRegManufacturerId = 0xFE;
  static constexpr uint8_t kRegDieId = 0xFF;
  static constexpr uint16_t kManufacturerId = 0x5449;  // "TI"
  static constexpr uint16_t kDieId = 0x2260;
  static constexpr double kShuntLsb = 2.5e-6;  // V
  static constexpr double kBusLsb = 1.25e-3;   // V

  // averages: 1, 4, 16, 64, 128, 256, 512 or 1024 (rounded down to one of them).
  // conversion_us: 140, 204, 332, 588, 1100, 2116, 4156 or 8244 (nearest below).
  static uint16_t configWord(int averages, int conversion_us);

  Ina226(std::unique_ptr<RegisterBus> bus, double shunt_ohms);

  // Checks the IDs and writes the configuration (continuous shunt + bus). Error text in `error`.
  bool init(uint16_t config, std::string & error);
  bool read(Reading & out);

private:
  std::unique_ptr<RegisterBus> bus_;
  double shunt_ohms_;
};

}  // namespace open_mower_next::ina226
