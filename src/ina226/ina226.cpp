#include "ina226/ina226.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace open_mower_next::ina226
{
namespace
{
class LinuxI2c : public RegisterBus
{
public:
  explicit LinuxI2c(int fd) : fd_(fd) {}
  ~LinuxI2c() override { ::close(fd_); }
  bool read16(uint8_t reg, uint16_t & value) override
  {
    uint8_t b[2];
    if (::write(fd_, &reg, 1) != 1 || ::read(fd_, b, 2) != 2) return false;
    value = static_cast<uint16_t>((b[0] << 8) | b[1]);
    return true;
  }
  bool write16(uint8_t reg, uint16_t value) override
  {
    uint8_t b[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return ::write(fd_, b, 3) == 3;
  }

private:
  int fd_;
};
}  // namespace

std::unique_ptr<RegisterBus> openLinuxI2c(const std::string & device, uint8_t address, std::string & error)
{
  const int fd = ::open(device.c_str(), O_RDWR);
  if (fd < 0) {
    error = "open " + device + ": " + std::strerror(errno);
    return nullptr;
  }
  if (::ioctl(fd, I2C_SLAVE, address) < 0) {
    error = "I2C_SLAVE: " + std::string(std::strerror(errno));
    ::close(fd);
    return nullptr;
  }
  return std::make_unique<LinuxI2c>(fd);
}

uint16_t Ina226::configWord(int averages, int conversion_us)
{
  static constexpr int kAvg[] = {1, 4, 16, 64, 128, 256, 512, 1024};
  static constexpr int kCt[] = {140, 204, 332, 588, 1100, 2116, 4156, 8244};
  uint16_t avg = 0, ct = 0;
  for (uint16_t i = 0; i < 8; ++i) {
    if (averages >= kAvg[i]) avg = i;
    if (conversion_us >= kCt[i]) ct = i;
  }
  // Bit 14 reads back as 1 (reserved); mode 7 = shunt and bus, continuous.
  return static_cast<uint16_t>(0x4000 | (avg << 9) | (ct << 6) | (ct << 3) | 0x7);
}

Ina226::Ina226(std::unique_ptr<RegisterBus> bus, double shunt_ohms)
: bus_(std::move(bus)), shunt_ohms_(shunt_ohms)
{
}

bool Ina226::init(uint16_t config, std::string & error)
{
  uint16_t mfg = 0, die = 0;
  if (!bus_->read16(kRegManufacturerId, mfg) || !bus_->read16(kRegDieId, die)) {
    error = "no answer";
    return false;
  }
  if (mfg != kManufacturerId || die != kDieId) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "not an INA226 (manufacturer 0x%04X, die 0x%04X)", mfg, die);
    error = buf;
    return false;
  }
  uint16_t readback = 0;
  if (!bus_->write16(kRegConfig, config) || !bus_->read16(kRegConfig, readback) || readback != config) {
    error = "configuration write failed";
    return false;
  }
  return true;
}

bool Ina226::read(Reading & out)
{
  uint16_t shunt = 0, bus = 0;
  if (!bus_->read16(kRegShunt, shunt) || !bus_->read16(kRegBus, bus)) return false;
  out.shunt_voltage = static_cast<int16_t>(shunt) * kShuntLsb;
  out.bus_voltage = bus * kBusLsb;
  out.current = out.shunt_voltage / shunt_ohms_;
  out.power = out.bus_voltage * out.current;
  return true;
}

}  // namespace open_mower_next::ina226
