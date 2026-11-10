#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stl50b2_driver/stl50b2_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node = std::make_shared<Stl50b2Node>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("stl50b2_driver"), "%s", error.what());

    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}