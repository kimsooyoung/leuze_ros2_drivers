#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanPublisher : public rclcpp::Node
{
public:
  LaserScanPublisher() : Node("laser_scan_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "Laser scan publisher has been started.");
    
    // Create a publisher for the LaserScan message
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    
    // Publish the LaserScan message periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&LaserScanPublisher::laser_pub, this)
    );
  }

  void laser_pub(){
    // Initialize the LaserScan message
    scan_msg_.header.frame_id = "laser_frame"; // Set the frame ID
    scan_msg_.header.stamp = this->get_clock()->now(); // Set the timestamp

    scan_msg_.angle_min = 0.0;                  // Minimum angle in radians
    scan_msg_.angle_max = 2 * M_PI;             // Maximum angle in radians
    scan_msg_.angle_increment = 2 * M_PI / 720; // Angular distance between measurements in radians
    scan_msg_.time_increment = 0.0;             // Time between measurements in seconds
    scan_msg_.scan_time = 0.1;                   // Time taken to complete a full scan in seconds
    scan_msg_.range_min = 0.0;                  // Minimum range value in meters
    scan_msg_.range_max = 10.0;                 // Maximum range value in meters


    scan_msg_.ranges.resize(0);
    scan_msg_.ranges.resize(2700);


    scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);
    scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);
    scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);
    scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);
    scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);
    scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);
    scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);
    scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);
    scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);
    scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);
    scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);
    scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.709);scan_msg_.ranges.push_back(2.705);
    scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.701);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.718);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.713);scan_msg_.ranges.push_back(2.722);scan_msg_.ranges.push_back(2.705);

    // // Populate the laser scan push_back(ize_t i) = 0; i < 720; ++i)
    // {
    //   scan_msg_.ranges.push_back(; //) Replace '1.0' with your desired laser scan range value
    // }

    std::cout << scan_msg_.ranges.size() << std::endl;


    publisher_->publish(scan_msg_); // Publish the LaserScan message
  }

private:
  sensor_msgs::msg::LaserScan scan_msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanPublisher>());
  rclcpp::shutdown();
  return 0;
}


// 2.722 
// 2.713 
// 2.722 
// 2.705 
// 2.713 
// 2.718 
// 2.718 
// 2.713 
// 2.722 
// 2.718 
// 2.718 
// 2.701 
// 2.705 
// 2.718 
// 2.701 
// 2.713 
// 2.713 
// 2.713 
// 2.713 
// 2.718 
// 2.709 
// 2.722 
// 2.713 
// 2.718 
// 2.713 
// 2.713 
// 2.718 
// 2.713 
// 2.709 
// 2.705 
// 2.718 
// 2.701 
// 2.722 
// 2.713 
// 2.713 
// 2.701 
// 2.713 
// 2.709 
// 2.713 
// 2.713 
// 2.718 
// 2.713 
// 2.713 
// 2.705 
// 2.713 
// 2.709 
// 2.718 
// 2.705 
// 2.705 
// 2.705 
// 2.701 
// 2.709 
// 2.697 
// 2.713 
// 2.709 
// 2.718 
// 2.709 
// 2.718 
// 2.713 
// 2.697 
// 2.713 
// 2.718 
// 2.722 
// 2.701 
// 2.718 
// 2.709 
// 2.709 
// 2.709 
// 2.709 
// 2.709 
// 2.718 
// 2.713 
// 2.722 
// 2.709 
// 2.709 
// 2.709 
// 2.713 
// 2.726 
// 2.709 
// 2.718 
// 2.713 
// 2.726 
// 2.718 
// 2.718 
// 2.721 
// 2.707 
// 2.704 
// 2.704 
// 2.696 
// 2.704 
// 2.692 
// 2.699 
// 2.703 
// 2.703 
// 2.715 
// 2.707 
// 2.725 
// 2.72 
// 2.765 
// 2.768 
// 2.777 
// 4.701 
// 4.776 
// 4.429 
// 4.429 
// 4.423 
// 4.419 
// 4.424 
// 4.432 
// 4.431 
// 4.416 
// 4.415 
// 4.42 
// 4.403 
// 4.424 
// 4.487 
// 4.572 
// 4.554 
// 4.558 
// 4.539 
// 4.548 
// 4.541 
// 4.563 
// 4.547 
// 4.551 
// 4.535 
// 4.531 
// 4.535 
// 4.559 
// 4.525 
// 4.522 
// 4.518 
// 4.51 
// 4.527 
// 4.515 
// 4.523 
// 4.518 
// 4.533 
// 4.534 
// 4.558 
// 4.557 
// 4.811 
// 4.811 
// 4.802 
// 4.81 
// 4.801 
// 4.825 
// 4.824 
// 4.848 
// 7.657 
// 7.773 
// 7.689 
// 7.649 
// 7.589 
// 7.576 
// 7.572 
// 7.575 
// 7.587 
// 7.597 
// 7.618 
// 7.66 
// 7.667 
// 7.647 
// 7.654 
// 7.651 
// 7.65 
// 7.65 
// 7.67 
// 7.674 
// 7.681 
// 7.695 
// 7.721 
// 65.535 
// 10.749 
// 65.535 
// 10.664 
// 65.535 
// 65.535 
// 10.656 
// 10.667 
// 10.662 
// 6.808 
// 6.796 
// 6.8 
// 6.782 
// 6.786 
// 6.702 
// 6.592 
// 6.403 
// 6.346 
// 6.338 
// 6.282 
// 6.249 
// 6.232 
// 6.175 
// 4.582 
// 4.558 
// 4.553 
// 4.528 
// 4.513 
// 4.489 
// 4.514 
// 4.49 
// 4.49 
// 4.477 
// 4.494 
// 4.478 
// 4.494 
// 4.49 
// 4.501 
// 4.506 
// 4.494 
// 4.515 
// 4.497 
// 4.5 
// 4.49 
// 4.512 
// 4.486 
// 2.099 
// 2.113 
// 2.095 
// 2.097 
// 2.092 
// 2.084 
// 2.095 
// 2.095 
// 2.081 
// 2.099 
// 2.089 
// 2.102 
// 2.108 
// 2.147 
// 2.167 
// 2.188 
// 2.21 
// 2.212 
// 2.237 
// 2.261 
// 2.29 
// 2.316 
// 2.339 
// 2.366 
// 2.377 
// 2.409 
// 2.464 
// 2.492 
// 2.509 
// 2.55 
// 2.566 
// 2.594 
// 2.624 
// 2.649 
// 2.685 
// 2.694 
// 2.739 
// 2.775 
// 2.8 
// 2.824 
// 2.866 
// 2.913 
// 2.936 
// 2.948 
// 2.946 
// 2.925 
// 2.925 
// 2.921 
// 2.931 
// 2.922 
// 2.922 
// 2.913 
// 2.925 
// 2.921 
// 2.929 
// 2.929 
// 2.938 
// 2.939 
// 2.933 
// 2.925 
// 2.905 
// 2.895 
// 2.887 
// 2.875 
// 2.867 
// 2.85 
// 2.847 
// 2.834 
// 2.818 
// 2.804 
// 2.812 
// 2.79 
// 2.773 
// 2.76 
// 2.758 
// 2.738 
// 2.738 
// 2.724 
// 2.711 
// 2.711 
// 2.69 
// 2.695 
// 2.67 
// 2.67 
// 2.637 
// 2.654 
// 2.041 
// 2.003 
// 2.017 
// 2.055 
// 2.588 
// 2.592 
// 2.579 
// 2.572 
// 2.571 
// 2.55 
// 2.01 
// 1.846 
// 1.855 
// 1.826 
// 1.822 
// 1.826 
// 1.81 
// 1.899 
// 1.973 
// 1.947 
// 1.958 
// 1.946 
// 1.949 
// 1.933 
// 1.929 
// 1.921 
// 1.917 
// 1.913 
// 1.9 
// 1.909 
// 1.892 
// 1.904 
// 1.896 
// 1.884 
// 1.872 
// 1.872 
// 1.872 
// 1.859 
// 1.863 
// 1.848 
// 1.843 
// 1.84 
// 1.839 
// 1.822 
// 1.833 
// 1.819 
// 1.826 
// 1.803 
// 1.81 
// 1.798 
// 1.803 
// 1.803 
// 1.787 
// 1.799 
// 1.779 
// 1.791 
// 1.776 
// 1.776 
// 1.768 
// 1.772 
// 1.755 
// 1.764 
// 1.764 
// 1.773 
// 1.785 
// 1.785 
// 1.8 
// 1.776 
// 1.768 
// 1.751 
// 1.784 
// 1.833 
// 1.866 
// 1.87 
// 1.862 
// 1.883 
// 1.875 
// 1.887 
// 1.879 
// 1.891 
// 1.898 
// 1.897 
// 1.896 
// 1.903 
// 1.898 
// 1.917 
// 0.924 
// 0.92 
// 0.916 
// 0.916 
// 0.915 
// 0.904 
// 0.91 
// 0.9 
// 0.906 
// 0.891 
// 0.901 
// 0.893 
// 0.894 
// 0.881 
// 0.892 
// 0.888 
// 0.896 
// 0.891 
// 0.904 
// 0.897 
// 0.897 
// 0.901 
// 0.904 
// 0.927 
// 0.926 
// 0.938 
// 0.946 
// 0.937 
// 0.938 
// 0.927 
// 0.91 
// 0.908 
// 0.898 
// 0.893 
// 0.9 
// 0.895 
// 0.91 
// 0.887 
// 0.895 
// 0.877 
// 0.83 
// 0.823 
// 0.813 
// 0.824 
// 0.809 
// 0.812 
// 0.823 
// 0.815 
// 0.797 
// 0.784 
// 0.807 
// 0.775 
// 0.802 
// 0.816 
// 0.773 
// 0.784 
// 0.798 
// 0.776 
// 0.767 
// 0.8 
// 0.781 
// 0.765 
// 0.762 
// 0.763 
// 0.762 
// 0.752 
// 0.767 
// 0.772 
// 0.77 
// 0.768 
// 0.776 
// 0.781 
// 0.773 
// 0.793 
// 0.773 
// 0.779 
// 0.769 
// 0.778 
// 0.787 
// 0.785 
// 0.789 
// 0.789 
// 0.791 
// 0.786 
// 0.781 
// 0.79 
// 0.782 
// 0.788 
// 0.776 
// 0.79 
// 0.785 
// 0.78 
// 0.775 
// 0.78 
// 0.775 
// 0.788 
// 0.783 
// 0.778 
// 0.786 
// 0.796 
// 0.778 
// 0.781 
// 0.777 
// 0.771 
// 0.772 
// 0.767 
// 0.758 
// 0.757 
// 0.76 
// 0.765 
// 0.758 
// 0.764 
// 0.747 
// 0.748 
// 0.735 
// 0.724 
// 0.709 
// 0.725 
// 0.708 
// 0.721 
// 0.699 
// 0.718 
// 0.697 
// 0.698 
// 0.702 
// 0.694 
// 0.71 
// 0.697 
// 0.707 
// 0.693 
// 0.707 
// 0.685 
// 0.705 
// 0.689 
// 0.691 
// 0.688 
// 0.699 
// 0.693 
// 0.699 
// 0.683 
// 0.671 
// 0.691 
// 0.703 
// 0.694 
// 0.697 
// 0.688 
// 0.693 
// 0.702 
// 0.711 
// 0.719 
// 0.727 
// 0.733 
// 0.723 
// 0.733 
// 0.732 
// 0.73 
// 0.735 
// 0.74 
// 0.739 
// 0.732 
// 0.746 
// 0.744 
// 0.75 
// 0.738 
// 0.741 
// 0.729 
// 0.742 
// 0.752 
// 0.734 
// 0.747 
// 0.744 
// 0.746 
// 0.739 
// 0.733 
// 0.741 
// 0.742 
// 0.741 
// 0.737 
// 0.742 
// 0.725 
// 0.75 
// 0.742 
// 0.747 
// 0.742 
// 0.737 
// 0.742 
// 0.749 
// 0.745 
// 0.739 
// 0.73 
// 0.742 
// 0.75 
// 0.753 
// 0.752 
// 0.75 
// 0.77 
// 0.766 
// 0.78 
// 0.777 
// 0.801 
// 0.801 
// 0.83 
// 0.922 
// 0.935 
// 0.929 
// 0.934 
// 0.94 
// 0.945 
// 0.949 
// 0.955 
// 0.958 
// 0.949 
// 0.958 
// 0.963 
// 0.962 
// 0.96 
// 0.963 
// 0.962 
// 0.964 
// 0.971 
// 0.971 
// 0.978 
// 1.001 
// 1.029 
// 1.038 
// 1.058 
// 1.038 
// 1.046 
// 1.038 
// 1.033 
// 1.029 
// 1.034 
// 1.04 
// 1.022 
// 0.999 
// 0.988 
// 0.983 
// 0.971 
// 0.972 
// 0.959 
// 0.964 
// 0.963 
// 0.958 
// 0.953 
// 0.963 
// 0.95 
// 0.959 
// 0.951 
// 0.95 
// 0.954 
// 0.951 
// 0.944 
// 0.95 
// 0.955 
// 0.943 
// 0.934 
// 0.945 
// 0.938 
// 0.938 
// 0.947 
// 0.951 
// 0.947 
// 0.946 
// 0.942 
// 0.942 
// 0.942 
// 0.96 
// 0.968 
// 0.977 
// 0.981 
// 0.997 
// 0.993 
// 0.991 
// 1.011 
// 1.016 
// 1.026 
// 1.025 
// 1.003 
// 1.006 
// 1.005 
// 1.006 
// 1 
// 1.023 
// 1.024 
// 1.03 
// 1.031 
// 1.026 
// 1.035 
// 1.023 
// 1.04 
// 1.04 
// 1.036 
// 1.048 
// 1.046 
// 1.045 
// 1.036 
// 1.039 
// 1.037 
// 1.036 
// 1.033 
// 1.034 
// 1.029 
// 1.029 
// 1.038 
// 1.039 
// 1.038 
// 1.038 
// 1.038 
// 1.021 
// 1.037 
// 1.028 
// 1.028 
// 1.02 
// 1.02 
// 1.028 
// 1.025 
// 1.021 
// 1.021 
// 1.017 
// 1.017 
// 1.017 
// 1.013 
// 1.021 
// 1.013 
// 1.017 
// 1.017 
// 1.013 
// 1.017 
// 1.005 
// 1.017 



// ros1