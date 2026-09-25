#pragma once
// SPI loop to the Worx board: one transfer per period, at most one queued
// command per transfer (like the ROS1 worx_comms), ping every ping_period so
// the firmware's SPI watchdog stays happy. Received messages are handed to the
// callback on the link thread.

#include "worx_hardware/transport.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace open_mower_next::worx_hardware
{

class WorxLink
{
public:
  struct Options
  {
    std::chrono::microseconds period{1000};
    std::chrono::milliseconds ping_period{2000};
    size_t max_queue = 64;  // oldest commands are dropped beyond this
  };
  using MessageCallback = std::function<void(const std::string &)>;

  WorxLink(std::unique_ptr<Transport> transport, Options options);
  ~WorxLink();

  bool start(MessageCallback on_message, std::string & error);
  void stop();

  // Queue a JSON command. urgent = jump the queue (speed/emergency).
  void send(const std::string & msg, bool urgent = false);

  // Seconds since the last received message (infinity before the first).
  double secondsSinceRx() const;
  uint64_t rxMessages() const { return rx_count_; }
  uint64_t rxDropped() const { return rx_dropped_; }
  uint64_t txTruncated() const { return tx_truncated_; }
  uint64_t transferErrors() const { return transfer_errors_; }

private:
  void run();

  std::unique_ptr<Transport> transport_;
  Options opt_;
  MessageCallback on_message_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex tx_mutex_;
  std::deque<std::string> tx_queue_;

  std::atomic<int64_t> last_rx_ns_{0};
  std::atomic<uint64_t> rx_count_{0}, rx_dropped_{0}, tx_truncated_{0}, transfer_errors_{0};
};

}  // namespace open_mower_next::worx_hardware
