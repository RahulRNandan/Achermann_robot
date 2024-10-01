#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class SMPCNode : public rclcpp::Node {
public:
  SMPCNode() : Node("smpc_node") {
    path_sub_ = this->create_subscription<std_msgs::msg::String>(
        "dijkstra_path", 10,
        std::bind(&SMPVNode::path_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_sub_;

  void path_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Following Path: %s", msg->data.c_str());
    // Implement SMPC logic to control the robot here
    // For demonstration, just print the path
  }

  void move_to_point(const std::pair<int, int> &point) {
    RCLCPP_INFO(this->get_logger(), "Moving to: (%d, %d)", point.first,
                point.second);
    // Implement SMPC control logic to adjust velocity and handle uncertainties
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SMPVNode>());
  rclcpp::shutdown();
  return 0;
}
