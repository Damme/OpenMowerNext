#include "web_ui/web_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>

#include <atomic>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <deque>
#include <map>
#include <thread>

namespace open_mower_next::web_ui
{

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;

std::optional<Cidr> Cidr::parse(const std::string & text)
{
  const auto slash = text.find('/');
  int bits = 32;
  if (slash != std::string::npos) {
    const std::string b = text.substr(slash + 1);
    if (b.empty() || b.size() > 2 || b.find_first_not_of("0123456789") != std::string::npos) return std::nullopt;
    bits = std::stoi(b);
    if (bits > 32) return std::nullopt;
  }
  in_addr a{};
  if (inet_pton(AF_INET, text.substr(0, slash).c_str(), &a) != 1) return std::nullopt;
  Cidr c;
  c.mask = bits == 0 ? 0 : ~uint32_t(0) << (32 - bits);
  c.net = ntohl(a.s_addr) & c.mask;
  return c;
}

class WsSession;

struct WebServer::Impl
{
  Options opt;
  MessageFn on_message;
  ConnectFn on_connect;
  LogFn log;
  std::atomic<bool> stopping{false};
  std::atomic<size_t> n_clients{0};
  // Everything below is touched on the io thread only.
  std::map<uint64_t, std::weak_ptr<WsSession>> sessions;
  uint64_t next_id = 1;
  size_t rejected = 0;
  std::chrono::steady_clock::time_point last_reject_log{};
  net::io_context ioc{1};
  tcp::acceptor acceptor{ioc};
  std::thread thread;

  bool allowed(const tcp::endpoint & ep) const
  {
    auto a = ep.address();
    if (a.is_v6() && a.to_v6().is_v4_mapped()) a = net::ip::make_address_v4(net::ip::v4_mapped, a.to_v6());
    if (!a.is_v4()) return false;
    const uint32_t v = a.to_v4().to_uint();
    for (const auto & c : opt.allow) {
      if (c.contains(v)) return true;
    }
    return false;
  }

  void accept();
  void connected(uint64_t id, bool up)
  {
    if (!stopping && on_connect) on_connect(id, up);
  }
  void message(uint64_t id, std::string text)
  {
    if (!stopping && on_message) on_message(id, std::move(text));
  }
};

class WsSession : public std::enable_shared_from_this<WsSession>
{
public:
  WsSession(tcp::socket && socket, WebServer::Impl * impl) : ws_(std::move(socket)), impl_(impl) {}
  ~WsSession()
  {
    if (!impl_->stopping) close();
  }

  void run(http::request<http::string_body> req)
  {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.read_message_max(256 * 1024);  // an area saved from the map editor (mm, thousands of points)
    ws_.auto_fragment(false);
    ws_.async_accept(req, [self = shared_from_this()](beast::error_code ec) {
      if (ec) return;
      self->id_ = self->impl_->next_id++;
      self->impl_->sessions[self->id_] = self;
      self->impl_->n_clients = self->impl_->sessions.size();
      self->open_ = true;
      self->impl_->connected(self->id_, true);
      self->read();
    });
  }

  void send(const std::shared_ptr<const std::string> & msg)
  {
    if (!open_) return;
    if (queued_ + msg->size() > impl_->opt.max_queued_bytes) {
      impl_->log(true, "web_ui: client " + std::to_string(id_) + " too slow, disconnected");
      close();
      beast::error_code ec;
      beast::get_lowest_layer(ws_).socket().close(ec);
      return;
    }
    queue_.push_back(msg);
    queued_ += msg->size();
    if (queue_.size() == 1) write();
  }

private:
  void read()
  {
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec, size_t) {
      if (ec) {
        self->close();
        return;
      }
      if (self->ws_.got_text()) self->impl_->message(self->id_, beast::buffers_to_string(self->buffer_.data()));
      self->buffer_.consume(self->buffer_.size());
      self->read();
    });
  }

  void write()
  {
    ws_.text(true);
    ws_.async_write(net::buffer(*queue_.front()), [self = shared_from_this()](beast::error_code ec, size_t) {
      if (ec) {
        self->close();
        return;
      }
      self->queued_ -= self->queue_.front()->size();
      self->queue_.pop_front();
      if (!self->queue_.empty()) self->write();
    });
  }

  void close()
  {
    if (!open_) return;
    open_ = false;
    impl_->sessions.erase(id_);
    impl_->n_clients = impl_->sessions.size();
    impl_->connected(id_, false);
  }

  websocket::stream<beast::tcp_stream> ws_;
  WebServer::Impl * impl_;
  beast::flat_buffer buffer_;
  std::deque<std::shared_ptr<const std::string>> queue_;
  size_t queued_ = 0;
  uint64_t id_ = 0;
  bool open_ = false;
};

class HttpSession : public std::enable_shared_from_this<HttpSession>
{
public:
  HttpSession(tcp::socket && socket, WebServer::Impl * impl) : stream_(std::move(socket)), impl_(impl) {}

  void read()
  {
    parser_.emplace();
    parser_->header_limit(8192);
    parser_->body_limit(1024);
    stream_.expires_after(std::chrono::seconds(15));
    http::async_read(stream_, buffer_, *parser_, [self = shared_from_this()](beast::error_code ec, size_t) {
      if (!ec) self->handle(self->parser_->release());
    });
  }

private:
  // Only the page itself may open the WebSocket: a browser sends the page's
  // origin, which must be this host. Tools without an Origin are fine.
  static bool sameOrigin(const http::request<http::string_body> & req)
  {
    const auto origin = req[http::field::origin];
    if (origin.empty()) return true;
    const std::string host(req[http::field::host]);
    return !host.empty() && (origin == "http://" + host || origin == "https://" + host);
  }

  void handle(http::request<http::string_body> && req)
  {
    if (websocket::is_upgrade(req)) {
      if (req.target() != "/ws" || !sameOrigin(req) || impl_->n_clients >= impl_->opt.max_clients) {
        reply(req, http::status::forbidden, "forbidden\n", "text/plain");
        return;
      }
      stream_.expires_never();
      std::make_shared<WsSession>(stream_.release_socket(), impl_)->run(std::move(req));
      return;
    }
    if (req.method() != http::verb::get && req.method() != http::verb::head) {
      reply(req, http::status::method_not_allowed, "method not allowed\n", "text/plain");
    } else if (req.target() == "/" || req.target() == "/index.html") {
      reply(req, http::status::ok, impl_->opt.page, "text/html; charset=utf-8");
    } else {
      reply(req, http::status::not_found, "not found\n", "text/plain");
    }
  }

  void reply(const http::request<http::string_body> & req, http::status status, const std::string & body,
             const char * type)
  {
    auto res = std::make_shared<http::response<http::string_body>>(status, req.version());
    res->set(http::field::content_type, type);
    res->set(http::field::cache_control, "no-store");
    res->set("X-Content-Type-Options", "nosniff");
    res->set("X-Frame-Options", "DENY");
    res->set("Referrer-Policy", "no-referrer");
    res->set("Content-Security-Policy",
             "default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src data:; "
             "connect-src 'self' " + wsSource(req) + "; base-uri 'none'; form-action 'none'; frame-ancestors 'none'");
    res->keep_alive(req.keep_alive());
    if (req.method() != http::verb::head) res->body() = body;
    res->prepare_payload();
    if (req.method() == http::verb::head) res->body().clear();
    http::async_write(stream_, *res, [self = shared_from_this(), res](beast::error_code ec, size_t) {
      if (ec || !res->keep_alive()) {
        self->stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
        return;
      }
      self->read();
    });
  }

  // Older Safari doesn't count ws:// as 'self': name this host explicitly
  // (only if the Host header is a plain host[:port]).
  static std::string wsSource(const http::request<http::string_body> & req)
  {
    const std::string host(req[http::field::host]);
    if (host.empty() || host.size() > 100 ||
        host.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:[]") !=
          std::string::npos) {
      return "";
    }
    return "ws://" + host;
  }

  beast::tcp_stream stream_;
  WebServer::Impl * impl_;
  beast::flat_buffer buffer_;
  std::optional<http::request_parser<http::string_body>> parser_;
};

void WebServer::Impl::accept()
{
  acceptor.async_accept([this](beast::error_code ec, tcp::socket socket) {
    if (ec == net::error::operation_aborted) return;
    if (!ec) {
      beast::error_code e;
      const auto peer = socket.remote_endpoint(e);
      if (!e && allowed(peer)) {
        std::make_shared<HttpSession>(std::move(socket), this)->read();
      } else {
        socket.close(e);
        ++rejected;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_reject_log > std::chrono::seconds(10)) {
          last_reject_log = now;
          log(true, "web_ui: refused " + std::to_string(rejected) + " connection(s), last from " +
                      (peer.address().is_unspecified() ? std::string("?") : peer.address().to_string()));
          rejected = 0;
        }
      }
    }
    accept();
  });
}

WebServer::WebServer(Options options, MessageFn on_message, ConnectFn on_connect, LogFn log)
  : impl_(std::make_shared<Impl>())
{
  impl_->opt = std::move(options);
  impl_->on_message = std::move(on_message);
  impl_->on_connect = std::move(on_connect);
  impl_->log = std::move(log);
}

WebServer::~WebServer()
{
  impl_->stopping = true;
  impl_->ioc.stop();
  if (impl_->thread.joinable()) impl_->thread.join();
  impl_->sessions.clear();
}

bool WebServer::start(std::string & error)
{
  beast::error_code ec;
  const auto addr = net::ip::make_address(impl_->opt.address, ec);
  if (ec) {
    error = "bad address '" + impl_->opt.address + "'";
    return false;
  }
  const tcp::endpoint ep(addr, impl_->opt.port);
  auto & acc = impl_->acceptor;
  acc.open(ep.protocol(), ec);
  if (!ec) acc.set_option(net::socket_base::reuse_address(true), ec);
  if (!ec && addr.is_v4()) {
    // The VPN interface may come up after us: bind its address anyway.
    const int one = 1;
    ::setsockopt(acc.native_handle(), IPPROTO_IP, IP_FREEBIND, &one, sizeof(one));
  }
  if (!ec) acc.bind(ep, ec);
  if (!ec) acc.listen(16, ec);
  if (ec) {
    error = ep.address().to_string() + ":" + std::to_string(ep.port()) + ": " + ec.message();
    return false;
  }
  impl_->accept();
  impl_->thread = std::thread([impl = impl_.get()]() {
    pthread_setname_np(pthread_self(), "web_ui");
    while (!impl->stopping) {
      try {
        impl->ioc.run();
        break;
      } catch (const std::exception & e) {
        impl->log(true, std::string("web_ui: ") + e.what());
      }
    }
  });
  return true;
}

void WebServer::send(uint64_t client, std::string text)
{
  auto msg = std::make_shared<const std::string>(std::move(text));
  net::post(impl_->ioc, [impl = impl_.get(), client, msg]() {
    if (client) {
      const auto it = impl->sessions.find(client);
      if (it == impl->sessions.end()) return;
      if (auto s = it->second.lock()) s->send(msg);
      return;
    }
    // send() may drop a slow client from the map: iterate over a copy.
    const auto all = impl->sessions;
    for (const auto & [id, weak] : all) {
      if (auto s = weak.lock()) s->send(msg);
    }
  });
}

size_t WebServer::clients() const
{
  return impl_->n_clients;
}

}  // namespace open_mower_next::web_ui
