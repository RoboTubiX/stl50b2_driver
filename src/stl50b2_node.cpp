#include "stl50b2_driver/stl50b2_node.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

Stl50b2Node::Stl50b2Node() : Node("stl50b2_node") {
  const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<std::string>("frame_id", "laser");
  clockwise_ = declare_parameter<bool>("clockwise", false);
  angle_offset_deg_ = declare_parameter<double>("angle_offset_deg", 0.0);

  range_min_ = static_cast<float>(declare_parameter<double>("range_min", 0.05));

  range_max_ = static_cast<float>(declare_parameter<double>("range_max", 12.0));

  if (range_min_ < 0.0F || range_max_ <= range_min_) {
    throw std::runtime_error("Invalid range_min/range_max");
  }

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  ranges_.assign(scan_bins_, std::numeric_limits<float>::infinity());
  intensities_.assign(scan_bins_, 0.0F);

  open_serial(port);
  start_motor();

  timer_ = create_wall_timer(5ms, [this]() { read_serial(); });
  timer_ = create_wall_timer(5ms, [this]() { read_serial(); });

  RCLCPP_INFO(get_logger(), "Opened %s at 230400 baud, 8N1", port.c_str());
}

Stl50b2Node::~Stl50b2Node() {
  if (timer_) {
    timer_->cancel();
  }

  if (serial_fd_ >= 0) {
    stop_motor();

    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void Stl50b2Node::fail_serial(const std::string& operation) {
  const int saved_errno = errno;

  ::close(serial_fd_);
  serial_fd_ = -1;

  throw std::runtime_error(operation + ": " + std::strerror(saved_errno));
}

void Stl50b2Node::open_serial(const std::string& port) {
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0) {
    throw std::runtime_error("Cannot open " + port + ": " + std::strerror(errno));
  }

  termios tty{};

  if (::tcgetattr(serial_fd_, &tty) != 0) {
    fail_serial("tcgetattr");
  }

  // Raw mode: giữ nguyên dữ liệu nhị phân.
  ::cfmakeraw(&tty);

  if (::cfsetispeed(&tty, B230400) != 0 || ::cfsetospeed(&tty, B230400) != 0) {
    fail_serial("Set baud rate");
  }

  // 8 data bits, no parity, 1 stop bit.
  tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CS8 | CLOCAL | CREAD;

  // Không dùng software flow control.
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    fail_serial("tcsetattr");
  }
}

void Stl50b2Node::read_serial() {
  std::array<unsigned char, 4096> buffer{};

  // Giới hạn số lượt đọc để callback không chiếm luồng quá lâu.
  for (int attempt = 0; attempt < 8; ++attempt) {
    const ssize_t count = ::read(serial_fd_, buffer.data(), buffer.size());

    if (count > 0) {
      received_bytes_ += static_cast<std::size_t>(count);

      process_bytes(buffer.data(), static_cast<std::size_t>(count));

      continue;
    }

    if (count == 0) {
      return;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return;
    }

    if (errno == EINTR) {
      continue;
    }

    RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));

    timer_->cancel();
    return;
  }
}
void Stl50b2Node::start_motor() {
  const std::array<unsigned char, 8> command{0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};

  std::size_t sent = 0;
  const auto deadline = std::chrono::steady_clock::now() + 500ms;

  while (sent < command.size()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      fail_start("Timeout sending start command");
    }

    const ssize_t count = ::write(serial_fd_, command.data() + sent, command.size() - sent);

    if (count > 0) {
      sent += static_cast<std::size_t>(count);
      continue;
    }

    if (count < 0 && errno == EINTR) {
      continue;
    }

    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      fail_serial("Send start command");
    }

    pollfd descriptor{};
    descriptor.fd = serial_fd_;
    descriptor.events = POLLOUT;

    const int result = ::poll(&descriptor, 1, 20);

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      fail_serial("poll");
    }

    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      fail_start("Serial disconnected while sending start command");
    }
  }

  RCLCPP_INFO(get_logger(), "Start command sent");
}

void Stl50b2Node::fail_start(const std::string& message) {
  ::close(serial_fd_);
  serial_fd_ = -1;
  throw std::runtime_error(message);
}
uint8_t Stl50b2Node::crc8(const uint8_t* data, std::size_t size) {
  uint8_t crc = 0;

  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x4D);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }

  return crc;
}

uint16_t Stl50b2Node::read_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                               (static_cast<uint16_t>(data[1]) << 8));
}

void Stl50b2Node::process_bytes(const unsigned char* data, std::size_t size) {
  rx_buffer_.insert(rx_buffer_.end(), data, data + size);

  while (rx_buffer_.size() >= 2) {
    // Bỏ dữ liệu điều khiển hoặc byte không thuộc header đo.
    if (rx_buffer_[0] != 0x54 || rx_buffer_[1] != 0x2C) {
      rx_buffer_.pop_front();
      continue;
    }

    // Giữ lại phần gói còn thiếu cho lần read tiếp theo.
    if (rx_buffer_.size() < 47) {
      return;
    }

    std::array<uint8_t, 47> packet{};

    for (std::size_t i = 0; i < packet.size(); ++i) {
      packet[i] = rx_buffer_[i];
    }

    if (crc8(packet.data(), 46) != packet[46]) {
      ++crc_errors_;

      // Dịch một byte để tìm lại header, tránh bỏ sót gói.
      rx_buffer_.pop_front();
      continue;
    }

    for (std::size_t i = 0; i < packet.size(); ++i) {
      rx_buffer_.pop_front();
    }

    ++valid_packets_;
    handle_packet(packet);
  }
}
void Stl50b2Node::handle_packet(const std::array<uint8_t, 47>& packet) {
  const double start_deg = read_u16_le(packet.data() + 4) / 100.0;

  const double end_deg = read_u16_le(packet.data() + 42) / 100.0;

  if (start_deg >= 360.0 || end_deg >= 360.0) {
    return;
  }

  const double span_deg = std::fmod(end_deg - start_deg + 360.0, 360.0);

  // Bỏ gói có khoảng góc bất thường khi motor đang khởi động.
  if (span_deg <= 0.0 || span_deg > 30.0) {
    return;
  }

  for (std::size_t i = 0; i < 12; ++i) {
    const double angle_deg = std::fmod(start_deg + span_deg * i / 11.0, 360.0);

    const bool wrapped = have_previous_angle_ && previous_angle_deg_ > 270.0 && angle_deg < 90.0;

    if (wrapped) {
      const auto current_time = std::chrono::steady_clock::now();

      if (collecting_scan_) {
        const double duration = std::chrono::duration<double>(current_time - scan_started_).count();

        publish_scan(duration);
      }

      std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());

      std::fill(intensities_.begin(), intensities_.end(), 0.0F);

      scan_stamp_ = now();
      scan_started_ = current_time;
      collecting_scan_ = true;
    }

    previous_angle_deg_ = angle_deg;
    have_previous_angle_ = true;

    if (!collecting_scan_) {
      continue;
    }

    const std::size_t offset = 6 + i * 3;

    const uint16_t distance_mm = read_u16_le(packet.data() + offset);

    const float distance_m = distance_mm / 1000.0F;

    if (distance_mm == 0 || distance_m < range_min_ || distance_m > range_max_) {
      continue;
    }

    double output_angle = (clockwise_ ? -angle_deg : angle_deg) + angle_offset_deg_;

    output_angle = std::fmod(output_angle, 360.0);

    if (output_angle < 0.0) {
      output_angle += 360.0;
    }

    const auto index = static_cast<std::size_t>(output_angle / 360.0 * scan_bins_);

    if (index >= scan_bins_) {
      continue;
    }

    // Nhiều điểm rơi vào cùng ô: giữ điểm gần nhất.
    if (distance_m < ranges_[index]) {
      ranges_[index] = distance_m;
      intensities_[index] = static_cast<float>(packet[offset + 2]);
    }
  }
}

void Stl50b2Node::publish_scan(double duration) {
  if (duration <= 0.0) {
    return;
  }

  sensor_msgs::msg::LaserScan scan;

  scan.header.stamp = scan_stamp_;
  scan.header.frame_id = get_parameter("frame_id").as_string();

  scan.angle_min = 0.0F;
  scan.angle_increment = static_cast<float>(2.0 * pi_ / scan_bins_);

  scan.angle_max = scan.angle_min + (scan_bins_ - 1) * scan.angle_increment;

  scan.scan_time = static_cast<float>(duration);

  // Các điểm đã được sắp lại theo ô góc.
  // Chưa cung cấp thời gian riêng cho từng tia.
  scan.time_increment = 0.0F;

  scan.range_min = range_min_;
  scan.range_max = range_max_;
  scan.ranges = ranges_;
  scan.intensities = intensities_;

  scan_pub_->publish(scan);

  const auto valid = std::count_if(ranges_.begin(), ranges_.end(),
                                   [](float value) { return std::isfinite(value); });

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "Scan: %zu/%zu bins, %.2f Hz | CRC errors=%zu",
                       static_cast<std::size_t>(valid), scan_bins_, 1.0 / duration, crc_errors_);
}
void Stl50b2Node::stop_motor() noexcept {
  if (serial_fd_ < 0) {
    return;
  }

  const std::array<unsigned char, 8> command{0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A};

  std::size_t sent = 0;

  const auto deadline = std::chrono::steady_clock::now() + 300ms;

  while (sent < command.size() && std::chrono::steady_clock::now() < deadline) {
    const ssize_t count = ::write(serial_fd_, command.data() + sent, command.size() - sent);

    if (count > 0) {
      sent += static_cast<std::size_t>(count);
      continue;
    }

    if (count < 0 && errno == EINTR) {
      continue;
    }

    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "Cannot send stop command: %s", std::strerror(errno));
      return;
    }

    pollfd descriptor{};
    descriptor.fd = serial_fd_;
    descriptor.events = POLLOUT;

    const int result = ::poll(&descriptor, 1, 20);

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      RCLCPP_WARN(get_logger(), "Stop command poll failed: %s", std::strerror(errno));
      return;
    }

    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      RCLCPP_WARN(get_logger(), "Serial disconnected before motor stop");
      return;
    }
  }

  if (sent == command.size()) {
    RCLCPP_INFO(get_logger(), "Stop command sent");
  } else {
    RCLCPP_WARN(get_logger(), "Timeout sending stop command");
  }
}