#include "leuze_rsl_driver/rsl400_interface.hpp"
#include <angles/angles.h>
#include <algorithm>

RSL400Interface::RSL400Interface(std::string address, std::string port):
  Node("leuze_driver"), HardwareInterface(address, port, this)
{
  //-135/135 0.1
  pub_scan_ = this->create_publisher<LaserScan>("scan", 50);
  pub_status_ = this->create_publisher<ExtendedStatusProfileMsg>("status", 50);
  configuration_received_ = false;
  scan_size_ = 2700;
  scan_data_.resize(scan_size_);

  this->declare_parameter("scan_frame", "world");
  this->declare_parameter("ls_debug", "false");

  auto scan_frame = this->get_parameter("scan_frame").as_string();
  auto ls_debug = this->get_parameter("ls_debug").as_string() == "true" ? true : false;
  
  RCLCPP_INFO(get_logger(), "scan_frame: %s", scan_frame.c_str());
  RCLCPP_INFO(get_logger(), "ls_debug: %s", ls_debug == true ? "true" : "false");

  header_frame_ = scan_frame;

  // if(!private_nh_.getParam("scan_frame",header_frame_)){
  //   ROS_WARN_STREAM("[Laser Scanner] scan_frame param not found, loading default value of \"world\"");
  //   header_frame_ = "world";
  // }

  if(ls_debug == true)
    debug_on = true;

  if(debug_on){
    pub_debug_ = this->create_publisher<String>("scan_raw_data", 50);

    RCLCPP_WARN(get_logger(), "[Laser Scanner] Debug Mode is on. This might affect the performace.");
  }
  // resetDefault();
}

RSL400Interface::~RSL400Interface()
{
  // disconnect();
}
