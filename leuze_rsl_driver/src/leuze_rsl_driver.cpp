#include "rclcpp/rclcpp.hpp"
// #include "leuze_rsl_driver/rsl400_interface.hpp"
#include "leuze_rsl_driver/my_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // TODO: into node param
  std::string address("192.168.10.1");
  std::string port("9990");

  // std::shared_ptr<rclcpp::Node> node{new RSL400Interface(address, port)};
  // std::shared_ptr<rclcpp::Node> node = std::make_shared<RSL400Interface>("192.168.10.1", "9990");
  // std::shared_ptr<rclcpp::Node> node = std::make_shared<RSL400Interface>(address, port);
  // auto node = std::make_shared<RSL400Interface>(address, port);
  auto node = std::make_shared<MyNode>(address, port);
  node->connect();
  
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}