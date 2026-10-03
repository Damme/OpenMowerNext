// GPS for the Worx robot: reads UBX from the F9P through str2str's TCP server
// (the Pi runs `str2str -in serial://ttyS0 -out tcpsvr://:5015 -b 1`; NTRIP
// corrections go in through a second str2str) and publishes NAV-PVT as
// sensor_msgs/NavSatFix on gps/fix. Read-only: with `-b 1` anything written to
// the socket would reach the receiver, so nothing is.
//
// Status: NO_FIX without gnssFixOK, GBAS_FIX for RTK fixed, FIX otherwise.
// Covariance: hAcc^2, hAcc^2, vAcc^2 (diagonal known).
//
// Optional (elevation.file set): RTK fixed heights averaged into a grid in the
// map frame (ElevationGrid), written as CSV every elevation.save_interval s if
// something changed, and on shutdown. For terrain in CAD.
#include "ubx_gps/elevation_grid.hpp"
#include "ubx_gps/ubx.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <GeographicLib/LocalCartesian.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

namespace open_mower_next::ubx_gps
{

class UbxGpsNode : public rclcpp::Node
{
public:
  explicit UbxGpsNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ubx_gps", options)
  {
    host_ = declare_parameter("host", std::string("127.0.0.1"));
    port_ = declare_parameter("port", 5015);
    frame_id_ = declare_parameter("frame_id", std::string("gps"));
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    initElevation();
    thread_ = std::thread([this]() { run(); });
  }

  ~UbxGpsNode() override
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (elevation_ && elevation_->dirty()) saveElevation();
  }

private:
  void initElevation()
  {
    elevation_file_ = declare_parameter("elevation.file", std::string(""));
    const auto datum = declare_parameter("elevation.datum", std::vector<double>{});
    const double cell = declare_parameter("elevation.cell", 0.2);
    elevation_max_v_acc_ = declare_parameter("elevation.max_v_acc", 0.03);
    elevation_antenna_height_ = declare_parameter("elevation.antenna_height", 0.0);
    elevation_min_samples_ = declare_parameter("elevation.min_samples", 3);
    const auto max_samples = declare_parameter("elevation.max_samples", 100);
    elevation_save_interval_ = std::chrono::seconds(declare_parameter("elevation.save_interval", 600));
    if (elevation_file_.empty()) return;
    if (datum.size() < 2 || cell <= 0.0 || max_samples < 1) {
      RCLCPP_ERROR(get_logger(), "elevation.file set but elevation.datum [lat, lon] / cell / max_samples invalid: grid off");
      return;
    }
    local_ = std::make_unique<GeographicLib::LocalCartesian>(datum[0], datum[1], 0.0);
    elevation_ = std::make_unique<ElevationGrid>(cell, static_cast<uint32_t>(max_samples));
    const int read = elevation_->load(elevation_file_);
    RCLCPP_INFO(get_logger(), "Elevation grid %.2f m -> %s (%d cells loaded)", cell, elevation_file_.c_str(),
                std::max(read, 0));
    last_elevation_save_ = std::chrono::steady_clock::now();
  }

  // GPS thread only (and the destructor after the join): no lock needed.
  void addElevation(const NavPvt & p)
  {
    if (!elevation_ || !p.fix_ok || p.carr_soln != 2 || p.v_acc_m > elevation_max_v_acc_) return;
    double x, y, z;
    local_->Forward(p.lat_deg, p.lon_deg, p.height_m, x, y, z);
    elevation_->add(x, y, p.h_msl_m - elevation_antenna_height_);
    geoid_sep_ = p.height_m - p.h_msl_m;
    const auto t = std::chrono::steady_clock::now();
    if (elevation_->dirty() && t - last_elevation_save_ >= elevation_save_interval_) {
      last_elevation_save_ = t;
      saveElevation();
    }
  }

  void saveElevation()
  {
    char header[256];
    std::snprintf(header, sizeof(header),
                  "# x,y: cell centre [m] east/north of the map datum, cell %.2f m\n"
                  "# z: height above mean sea level (receiver geoid; ellipsoid = z + %.3f) minus antenna "
                  "%.3f m, n samples, std [m]\n",
                  elevation_->cell(), geoid_sep_, elevation_antenna_height_);
    if (!elevation_->save(elevation_file_, header, static_cast<uint32_t>(elevation_min_samples_))) {
      RCLCPP_WARN(get_logger(), "Cannot write %s", elevation_file_.c_str());
    }
  }

  bool running() const { return rclcpp::ok() && !stop_; }

  int connectOnce()
  {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * res = nullptr;
    if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) return -1;
    const int fd = ::socket(res->ai_family, res->ai_socktype, 0);
    if (fd >= 0 && ::connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
      ::close(fd);
      freeaddrinfo(res);
      return -1;
    }
    freeaddrinfo(res);
    return fd;
  }

  void run()
  {
    FrameParser parser([this](uint8_t cls, uint8_t id, const uint8_t * p, size_t len) {
      if (cls == 0x01 && id == 0x07) {
        if (auto pvt = parseNavPvt(p, len)) publish(*pvt);
      }
    });
    while (running()) {
      const int fd = connectOnce();
      if (fd < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot connect to %s:%ld, retrying",
                             host_.c_str(), port_);
        for (int i = 0; i < 20 && running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      RCLCPP_INFO(get_logger(), "Connected to %s:%ld", host_.c_str(), port_);
      auto last_rx = std::chrono::steady_clock::now();
      uint8_t buf[4096];
      while (running()) {
        pollfd pfd{fd, POLLIN, 0};
        const int r = ::poll(&pfd, 1, 200);
        if (r > 0) {
          const ssize_t n = ::read(fd, buf, sizeof(buf));
          if (n <= 0) break;
          parser.feed(buf, static_cast<size_t>(n));
          last_rx = std::chrono::steady_clock::now();
        } else if (r < 0 || std::chrono::steady_clock::now() - last_rx > std::chrono::seconds(5)) {
          RCLCPP_WARN(get_logger(), "No data for 5 s, reconnecting");
          break;
        }
      }
      ::close(fd);
    }
  }

  void publish(const NavPvt & p)
  {
    sensor_msgs::msg::NavSatFix m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                       sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    if (!p.fix_ok || p.fix_type < 2 || p.fix_type > 4) {
      m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (p.carr_soln == 2) {
      m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else {
      m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    m.latitude = p.lat_deg;
    m.longitude = p.lon_deg;
    m.altitude = p.height_m;
    m.position_covariance[0] = m.position_covariance[4] = p.h_acc_m * p.h_acc_m;
    m.position_covariance[8] = p.v_acc_m * p.v_acc_m;
    m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub_->publish(m);
    addElevation(p);

    const int quality = !p.fix_ok ? -1 : p.carr_soln;
    if (quality != last_quality_) {
      static const char * names[] = {"no fix", "fix (no RTK)", "RTK float", "RTK fixed"};
      RCLCPP_INFO(get_logger(), "GPS: %s, %u satellites, hAcc %.3f m", names[quality + 1], p.num_sv, p.h_acc_m);
      last_quality_ = quality;
    }
  }

  std::string host_;
  int64_t port_;
  std::string frame_id_;
  int last_quality_ = -2;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  std::string elevation_file_;
  double elevation_max_v_acc_ = 0.03;
  double elevation_antenna_height_ = 0.0;
  int64_t elevation_min_samples_ = 3;
  std::chrono::seconds elevation_save_interval_{600};
  std::chrono::steady_clock::time_point last_elevation_save_;
  double geoid_sep_ = 0.0;
  std::unique_ptr<GeographicLib::LocalCartesian> local_;
  std::unique_ptr<ElevationGrid> elevation_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace open_mower_next::ubx_gps

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::ubx_gps::UbxGpsNode)
