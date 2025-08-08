#include "rclcpp/rclcpp.hpp"

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    RCLCPP_INFO(this->get_logger(), "Controller node started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
