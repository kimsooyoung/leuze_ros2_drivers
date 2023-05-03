#ifndef LEUZE_RSL400_INTERFACE_H
#define LEUZE_RSL400_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "leuze_rsl_driver/communication.hpp"
#include "leuze_rsl_driver/hardware_interface.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "leuze_msgs/msg/extended_status_profile_msg.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using ExtendedStatusProfileMsg = leuze_msgs::msg::ExtendedStatusProfileMsg;
using String = std_msgs::msg::String;

class RSL400Interface : public rclcpp::Node, HardwareInterface<UDPConnection>, DataParser
{
public:
  RSL400Interface();
  ~RSL400Interface();

  // void connect();
  // void disconnect();
  // int parseBuffer(std::basic_string<unsigned char> buffer);

protected:
  // void resetDefault();
  // bool checkScan();
  // void publishScan();
  // void verifyConfiguration(DatagramExtendedStatusProfile d_esp);
  // DatagramExtendedStatusProfile parseExtendedStatusProfile(std::basic_string<unsigned char> buffer);
  // DatagramMeasurementDataType parseScanData(std::basic_string<unsigned char> buffer, Frame* frame);
  // uint16_t convertBytesToUint16(unsigned char low_byte, unsigned char high_byte);
  // bool compareTwoFloats(float a, float b,float epsilon = 0.0001);

private:
  DataParser parser_;
  HardwareInterface<UDPConnection> hw_interface_;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<ExtendedStatusProfileMsg>::SharedPtr pub_status_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_;
  
  std::string header_frame_;
  bool configuration_received_;
  bool debug_on;

  LaserScan laser_scan_;
  ExtendedStatusProfileMsg status_msg_;

  int scan_number_;
  int configuration_type_; //type 3 = Distance + Intensity / type6 = Distance
  int measure_counter_;
  int block_counter_;
  int scan_size_;

  std::vector<DatagramMeasurementDataType> scan_data_;

  void LogBufferToDebug(std::basic_string<unsigned char> buffer);
  //HTTP protocol object
};


#endif
