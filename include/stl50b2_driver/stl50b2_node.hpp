#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Stl50b2Node : public rclcpp::Node {
 public:
  Stl50b2Node();
  ~Stl50b2Node() override;

 private:
  using Packet = std::array<uint8_t, 47>;

  static constexpr std::size_t scan_bins_ = 720;
  static constexpr double pi_ = 3.14159265358979323846;

  // Serial và điều khiển motor.
  void open_serial(const std::string& port);
  void read_serial();
  void start_motor();
  void stop_motor() noexcept;

  void fail_serial(const std::string& operation);
  void fail_start(const std::string& message);

  // Giải mã giao thức.
  static uint8_t crc8(const uint8_t* data, std::size_t size);
  static uint16_t read_u16_le(const uint8_t* data);

  void process_bytes(const unsigned char* data, std::size_t size);
  void handle_packet(const Packet& packet);

  // Ghép và xuất vòng quét.
  void publish_scan(double duration);

  int serial_fd_{-1};
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<uint8_t> rx_buffer_;
  std::size_t received_bytes_{0};
  std::size_t valid_packets_{0};
  std::size_t crc_errors_{0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::vector<float> ranges_;
  std::vector<float> intensities_;

  bool collecting_scan_{false};
  bool have_previous_angle_{false};
  bool clockwise_{false};

  double previous_angle_deg_{0.0};
  double angle_offset_deg_{0.0};

  float range_min_{0.05F};
  float range_max_{12.0F};

  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point scan_started_;
};