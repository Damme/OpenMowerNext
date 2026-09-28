// Small HTTP + WebSocket server for the web UI (Boost.Beast, one thread).
//
// Serves one embedded page on "/" and a WebSocket on "/ws". Only peers inside
// the allowed IPv4 networks get an answer; everyone else is disconnected right
// after accept. A browser WebSocket must come from the page itself (Origin ==
// Host), so a foreign web page open in the same browser cannot drive the robot.
//
// ROS-free: the node (web_ui_node.cpp) feeds it text and gets text back.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace open_mower_next::web_ui
{

struct Cidr
{
  uint32_t net = 0;   // host byte order
  uint32_t mask = 0;

  // "10.99.99.0/24", "127.0.0.1" (= /32). nullopt when malformed.
  static std::optional<Cidr> parse(const std::string & text);
  bool contains(uint32_t addr) const { return (addr & mask) == net; }
};

class WebServer
{
public:
  struct Options
  {
    std::string address = "127.0.0.1";  // bound even before the interface exists (IP_FREEBIND)
    uint16_t port = 8090;
    std::vector<Cidr> allow;            // empty = nobody
    std::string page;                   // served on "/"
    size_t max_clients = 4;
    size_t max_queued_bytes = 512 * 1024;  // per client; a slower client is dropped
  };
  using MessageFn = std::function<void(uint64_t client, std::string text)>;
  using ConnectFn = std::function<void(uint64_t client, bool connected)>;
  using LogFn = std::function<void(bool error, const std::string & text)>;

  WebServer(Options options, MessageFn on_message, ConnectFn on_connect, LogFn log);
  ~WebServer();
  WebServer(const WebServer &) = delete;
  WebServer & operator=(const WebServer &) = delete;

  // Binds and starts the thread. false + error when the socket can't be bound.
  bool start(std::string & error);

  // Thread-safe. client 0 = all clients.
  void send(uint64_t client, std::string text);
  size_t clients() const;

  struct Impl;

private:
  std::shared_ptr<Impl> impl_;
};

}  // namespace open_mower_next::web_ui
