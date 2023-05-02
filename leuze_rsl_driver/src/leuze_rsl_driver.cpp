#include "rclcpp/rclcpp.hpp"
#include "leuze_rsl_driver/rsl400_interface.hpp"

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "leuze_driver");
//     ros::NodeHandle n;
//     if (argc < 3)
//     {
//         std::cerr << "Not enough arguments!" << std::endl;
//     }
//     std::string address = argv[1];
//     std::string port = argv[2];

//     RSL400Interface rsl_interface(address, port, &n);
//     rsl_interface.connect();

//     ros::spin();
//     return 0;
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<RSL400Interface>("192.168.10.1", "9990");
  // rclcpp::SharedPtr node = std::make_shared<RSL400Interface>("192.168.10.1", "9990");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}