#include "rclcpp/rclcpp.hpp"

// CAUTION 
// currently, this project is written by header only 
#include "leuze_rsl_driver/rsl400_interface.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 3)
  {
      std::cerr << "Not enough arguments!" << std::endl;
  }
  std::string address = argv[1];
  std::string port = argv[2];

  std::cout << "address: " << address << std::endl;
  std::cout << "port: " << port << std::endl;

  // TODO: header seperation
  auto node = std::make_shared<MyNode>(address, port);
  node->connect();
  
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}