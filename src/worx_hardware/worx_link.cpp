#include "worx_hardware/worx_link.hpp"

#include <limits>
#include <vector>

namespace open_mower_next::worx_hardware
{

WorxLink::WorxLink(std::unique_ptr<Transport> transport, Options options)
: transport_(std::move(transport)), opt_(options)
{
}

WorxLink::~WorxLink() { stop(); }

bool WorxLink::start(MessageCallback on_message, std::string & error)
{
  if (running_) {
    return true;
  }
  if (!transport_ || !transport_->open(error)) {
    return false;
  }
  on_message_ = std::move(on_message);
  running_ = true;
  thread_ = std::thread(&WorxLink::run, this);
  return true;
}

void WorxLink::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void WorxLink::send(const std::string & msg, bool urgent)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (urgent) {
    tx_queue_.push_front(msg);
  } else {
    tx_queue_.push_back(msg);
  }
  while (tx_queue_.size() > opt_.max_queue) {
    tx_queue_.pop_back();
  }
}

double WorxLink::secondsSinceRx() const
{
  const int64_t last = last_rx_ns_.load();
  if (last == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  return static_cast<double>(now - last) * 1e-9;
}

void WorxLink::run()
{
  FrameDecoder decoder;
  Frame tx, rx;
  std::vector<std::string> msgs;
  int ping_count = 0;
  auto next_ping = std::chrono::steady_clock::now();
  auto next = std::chrono::steady_clock::now();

  while (running_) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_ping) {
      next_ping = now + opt_.ping_period;
      send(cmdPing(++ping_count));
    }

    std::string msg;
    {
      std::lock_guard<std::mutex> lock(tx_mutex_);
      if (!tx_queue_.empty()) {
        msg = std::move(tx_queue_.front());
        tx_queue_.pop_front();
      }
    }
    if (msg.empty()) {
      tx.fill(kNop);
    } else if (!encodeFrame(msg, tx)) {
      ++tx_truncated_;
    }

    rx.fill(kNop);
    if (!transport_->transfer(tx, rx)) {
      ++transfer_errors_;
    } else {
      msgs.clear();
      decoder.feed(rx.data(), rx.size(), msgs);
      rx_dropped_ = decoder.dropped();
      for (const auto & m : msgs) {
        ++rx_count_;
        last_rx_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
        if (on_message_) {
          on_message_(m);
        }
      }
    }

    next += opt_.period;
    const auto after = std::chrono::steady_clock::now();
    if (next < after) {
      next = after;  // fell behind (slow transfer): don't burst
    }
    std::this_thread::sleep_until(next);
  }
}

}  // namespace open_mower_next::worx_hardware
