#include "leuze_rsl_driver/rsl400_interface.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <ros/console.h>

RSL400Interface::RSL400Interface(std::string address, std::string port):
  HardwareInterface(address, port, this)
{
  //-135/135 0.1
  pub_scan_ = this->create_publisher<LaserScan>("scan", 50);
  pub_status_ = this->create_publisher<ExtendedStatusProfileMsg>("status", 50);
  configuration_received_ = false;
  scan_size_ = 2700;
  scan_data_.resize(scan_size_);

  this->declare_parameter("scan_frame", "world");
  this->declare_parameter("ls_debug", "false");
  this->declare_parameter("angle_min", -2.35619449);
  this->declare_parameter("angle_max", 2.35619449);
  this->declare_parameter("scan_time", 0.04);
  this->declare_parameter("range_min", 0.001);
  this->declare_parameter("range_max", 65.0);

  auto scan_frame = this->get_parameter("scan_frame").as_string();
  auto ls_debug = this->get_parameter("ls_debug").as_string() == "true" ? true : false;
  
  auto angle_min = this->get_parameter("angle_min").as_double();
  auto angle_max = this->get_parameter("angle_max").as_double();
  auto scan_time = this->get_parameter("scan_time").as_double();
  auto range_min = this->get_parameter("range_min").as_double();
  auto range_max = this->get_parameter("range_max").as_double();

  RCLCPP_INFO(get_logger(), "scan_frame: %s", scan_frame.c_str());
  RCLCPP_INFO(get_logger(), "ls_debug: %s", ls_debug.c_str());

  RCLCPP_INFO(get_logger(), "angle_min: %f", angle_min);
  RCLCPP_INFO(get_logger(), "angle_max: %f", angle_max);
  RCLCPP_INFO(get_logger(), "scan_time: %f", scan_time);
  RCLCPP_INFO(get_logger(), "range_min: %f", range_min);
  RCLCPP_INFO(get_logger(), "range_max: %f", range_max);


  header_frame_ = str_param.as_string();

  // if(!private_nh_.getParam("scan_frame",header_frame_)){
  //   ROS_WARN_STREAM("[Laser Scanner] scan_frame param not found, loading default value of \"world\"");
  //   header_frame_ = "world";
  // }

  if(ls_debug == true)
    debug_on = true;

  if(debug_on){
    pub_debug_ = this->create_publisher<String>("scan_raw_data", 50);

    RCLCPP_WARN(get_logger(), "[Laser Scanner] Debug Mode is on. This might affect the performace.");
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    // {
    //     ros::console::notifyLoggerLevelsChanged();
    //     ROS_DEBUG_STREAM("[Laser Scanner] Debug Stream Enabled");
    // }
  }
  resetDefault();
}

RSL400Interface::~RSL400Interface()
{
  disconnect();
}

void RSL400Interface::connect()
{
  HardwareInterface<UDPConnection>::connect();
  ROS_INFO_STREAM("[Laser Scanner] Listening to data");
}


void RSL400Interface::resetDefault()
{
  laser_scan_.header.frame_id = header_frame_;
  laser_scan_.angle_min = nh_.param("/angle_min",-2.35619449); //Default min value
  laser_scan_.angle_max = nh_.param("/angle_max",2.35619449); //Default min value
  laser_scan_.angle_increment = (laser_scan_.angle_max-laser_scan_.angle_min)/(float)scan_size_; //default max resolution
  laser_scan_.scan_time = nh_.param("/scan_time",0.04); // Default
  laser_scan_.range_min = nh_.param("/range_min",0.001); //default
  laser_scan_.range_max = nh_.param("/range_max",65.0);  //Max range 65m
  laser_scan_.ranges.resize(0);
  laser_scan_.ranges.resize(scan_size_);
  laser_scan_.intensities.resize(0);
  laser_scan_.intensities.resize(scan_size_);
  block_counter_ = 0;
  measure_counter_ = 0;
  scan_number_ = -1; // Get last scan number
  ROS_INFO_STREAM("[Laser Scanner] Reset data");
}

void RSL400Interface::disconnect()
{
  HardwareInterface<UDPConnection>::disconnect();
  ROS_INFO_STREAM("[Laser Scanner] RSL Disconnected");
}

int RSL400Interface::parseBuffer(std::basic_string<unsigned char> buffer)
{
  if(debug_on)
  {
    LogBufferToDebug(buffer);
  }
  Frame *frame = reinterpret_cast<Frame *>((char *)buffer.c_str());
  if(frame->id==1)
    // Extended status profile. Status profile + measurement contour descritpion. Pg8 3.3.1.
  {
    parseExtendedStatusProfile(buffer);
  }
  else if(frame->id==3 or frame->id==6)
  {
    if(configuration_received_ == false)
    {
       ROS_WARN_STREAM("[Laser Scanner] Scan data header not received, skipping measurement.");
    }
    else
    {
      if(frame->scan_number != scan_number_)
      {
        ROS_WARN_STREAM("[Laser Scanner] Unexpected Scan Data id, skipping measurement.");
      }
      else
      {
        scan_data_[frame->block] = parseScanData(buffer, frame);
      }
    }

  }
  else if(frame->id==0)
  {
    return frame->id;
  }
  else
  {
    ROS_ERROR_STREAM("[Laser Scanner] Unknown ID : " << frame->id);
    return -1;
  }
  if(measure_counter_ == scan_size_)
  {
    if(checkScan())
    {
      publishScan();
    }
  }
  if(measure_counter_ >= scan_size_)
  {
    ROS_WARN_STREAM("[Laser Scanner] Scan measure counter overflowed, resetting");
    configuration_received_ = false;
    resetDefault();
  }

  return frame->id;
}

DatagramExtendedStatusProfile RSL400Interface::parseExtendedStatusProfile(std::basic_string<unsigned char> buffer)
{
  DatagramExtendedStatusProfile *esp = reinterpret_cast<DatagramExtendedStatusProfile *>((char *)buffer.c_str());

  if(buffer.length() != esp->frame.h1.total_length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Extended Status Profile of incorrect length " << buffer.length() << ", expected " << esp->frame.h1.total_length);
    return *esp;
  }
  verifyConfiguration(*esp);

  status_msg_.header.frame_id = header_frame_;
  status_msg_.header.stamp = ros::Time::now();

  status_msg_.byte_0 = esp->status_profile.byte_0;
  status_msg_.byte_1 = esp->status_profile.byte_1;
  status_msg_.msg_and_ossd_2 = esp->status_profile.msg_and_ossd_2;
  status_msg_.emergency_stop_3 = esp->status_profile.emergency_stop_3;
  status_msg_.electrical_signals_byte_4 = esp->status_profile.electrical_signals_byte_4;
  status_msg_.electrical_signals_byte_5 = esp->status_profile.electrical_signals_byte_5;
  status_msg_.electrical_signals_byte_6 = esp->status_profile.electrical_signals_byte_6;
  status_msg_.electrical_signals_byte_7 = esp->status_profile.electrical_signals_byte_7;
  status_msg_.scan_number = esp->status_profile.scan_number;
  status_msg_.protec_func_a_12 = esp->status_profile.protec_func_a_12;
  status_msg_.fp_sel_a_byte_13 = esp->status_profile.fp_sel_a_byte_13;
  status_msg_.fp_sel_a_byte_14 = esp->status_profile.fp_sel_a_byte_14;
  status_msg_.indic_a_15 = esp->status_profile.indic_a_15;
  status_msg_.protec_func_b_16 = esp->status_profile.protec_func_b_16;
  status_msg_.fp_sel_b_byte_17 = esp->status_profile.fp_sel_b_byte_17;
  status_msg_.fp_sel_b_byte_18 = esp->status_profile.fp_sel_b_byte_18;
  status_msg_.indic_b_19 = esp->status_profile.indic_b_19;

  status_msg_.start_index = esp->measurement_contour_descritption.start_index;
  status_msg_.stop_index = esp->measurement_contour_descritption.stop_index;
  status_msg_.index_interval = esp->measurement_contour_descritption.index_interval;
  status_msg_.reserved = esp->measurement_contour_descritption.reseved;

  return *esp;
}

DatagramMeasurementDataType RSL400Interface::parseScanData(std::basic_string<unsigned char> buffer,
                                                           Frame* frame)
{
  DatagramMeasurementDataType mdt;
  int length = 0;
  mdt.frame = frame;
  if (frame->id==3){
    length=(frame->h1.total_length - 20)/4;
    for (int i = 20; i< buffer.length()  ; i+=4 )   // Capturing 4 bytes at a time - 2 for distance and 2 for signal strength as per UDP spec pg 14 table 3.6
    {
      uint16_t distance = convertBytesToUint16(buffer[i], buffer[i+1]);
      uint16_t intensity = convertBytesToUint16(buffer[i+2], buffer[i+3]);
      mdt.data_distance.push_back(distance);
      mdt.data_signal_strength.push_back(intensity);
    }
  }
  else if (frame->id==6){
    length=(frame->h1.total_length - 20)/2;
    for (int i = 20; i< buffer.length()  ; i+=2 )   // Capturing 2 bytes at a time for distance
    {
      uint16_t distance = convertBytesToUint16(buffer[i], buffer[i+1]);
      mdt.data_distance.push_back(distance);
      mdt.data_signal_strength.push_back(0.0);
    }
  }

  // Buffer length should match length declared by header
  if(mdt.frame->h1.total_length!=buffer.length())
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect length " << buffer.length() << ", expected " << mdt.frame->h1.total_length);
    return mdt;
  }
  // Number of distance/signal values is equal to data length in bytes/4 (because 4 bytes per value)
  // Data langth is total length of datagram - 20 (which is fixed frame size)
  // Refer UDP specs pg 14 3.3.2.2. This gives number of "measurement data values"/"scan values".
  if(mdt.data_distance.size() != length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect number of data values " << mdt.data_distance.size() << ", expected " << length);
    return mdt;
  }
  if(mdt.data_signal_strength.size() != length)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Scan data message of incorrect number of signal strength values " << mdt.data_signal_strength.size() << ", expected " << length);
    return mdt;
  }

  measure_counter_ += length;
  block_counter_ = mdt.frame->block+1;
  return mdt;
}


uint16_t RSL400Interface::convertBytesToUint16(unsigned char low_byte, unsigned char high_byte)
{
  return high_byte << 8 | low_byte;
}

bool RSL400Interface::compareTwoFloats(float a, float b, float epsilon)
{
  return fabs(a - b) < epsilon;
}

bool RSL400Interface::checkScan()
{
  int i_measure = 0;
  //Assemble data from block;
  for(int i_block=0; i_block < block_counter_; i_block++)
  {
    if(scan_data_[i_block].data_distance.size()==0)
    {
      ROS_INFO_STREAM("[Laser Scanner] Received scan data datagram with no distance values");
      return false;
    }
    else
    {
      for(int i_scan=0; i_scan< scan_data_[i_block].data_distance.size(); i_scan++)
      {
        laser_scan_.ranges[i_measure] = (float)scan_data_[i_block].data_distance[i_scan]/1000.0;
        laser_scan_.intensities[i_measure] = (float)scan_data_[i_block].data_signal_strength[i_scan];
        i_measure++;
      }
    }
  }
  laser_scan_.header.stamp = ros::Time::now();
  // Reverse the scans to match real world
  std::reverse(laser_scan_.ranges.begin(), laser_scan_.ranges.end());
  std::reverse(laser_scan_.intensities.begin(), laser_scan_.intensities.end());
  return true;
}
void RSL400Interface::publishScan()
{
  pub_scan_.publish(laser_scan_);
  pub_status_.publish(status_msg_);
  measure_counter_ =0;
}

void RSL400Interface::verifyConfiguration(DatagramExtendedStatusProfile d_esp)
{
  float min_angle_from_esp = ((float)(d_esp.measurement_contour_descritption.start_index)/10);
  //+1 here to account for the fact that internal calculations are for example from -135° to +135° but actual represntation is from 0° to 269.9° (difference of 0.1°)
  float max_angle_from_esp = ((float)(d_esp.measurement_contour_descritption.stop_index + 1)/10);
  float avg_angle = (min_angle_from_esp+max_angle_from_esp)/2;

  // Adjust for example from -135° to +135° to 0° to 270°
  min_angle_from_esp =  angles::from_degrees(min_angle_from_esp - avg_angle);
  max_angle_from_esp =  angles::from_degrees(max_angle_from_esp - avg_angle);

  if(!compareTwoFloats(min_angle_from_esp, laser_scan_.angle_min))
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal minimum angle of " << laser_scan_.angle_min << " does not match the value received from the laser " << min_angle_from_esp << ". Adjusting internally");
    laser_scan_.angle_min = min_angle_from_esp;
  }

  if(!compareTwoFloats(max_angle_from_esp,laser_scan_.angle_max))
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal maximum angle of " << laser_scan_.angle_max << " does not match the value received from the laser " << max_angle_from_esp << ". Adjusting internally");
    laser_scan_.angle_max = max_angle_from_esp;
  }

  if(scan_size_!=d_esp.getBeamCount())
  {
    ROS_WARN_STREAM("[Laser Scanner] Current internal beam count of " << scan_size_ << " does not match the value received from the laser " << d_esp.getBeamCount() << ". Adjusting internally");
    scan_size_ = d_esp.getBeamCount();
  }
  if(measure_counter_ != 0)
  {
    ROS_WARN_STREAM("[Laser Scanner] Received ExtendedProfile at unexcepted timing.");
    measure_counter_ =0;
  }
  scan_data_.clear();
  scan_data_.resize(scan_size_);
  scan_number_ = d_esp.frame.scan_number;
  configuration_received_ = true;
  // Length 48 is fixed
  if(d_esp.frame.h1.total_length!=48)
  {
    ROS_ERROR_STREAM("[Laser Scanner] Parsing Extended Status Profile of incorrect length " << d_esp.frame.h1.total_length << ", expected " << 48);
  }

}

void RSL400Interface::LogBufferToDebug(std::basic_string<unsigned char> buffer)
{
  std::stringstream oss;
  oss << std::hex;
  for(int i=0; i<buffer.length(); i++)
  {
    oss << std::setw(2) << std::setfill('0') << (int)(u_int16_t)buffer[i];
  }
  std_msgs::String string_msg;
  string_msg.data = oss.str();
  pub_debug_.publish(string_msg);
}
