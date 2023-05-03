#include "rclcpp/rclcpp.hpp"

class Sth {
private:
  std::string name_;

public:
  Sth(std::string name){
    name_ = name;
  }
};

class MyNode : public rclcpp::Node, Sth
{
public:
  MyNode(std::string name) : Node("my_node"), Sth(name)
  {
    // Constructor logic here
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MyNode>("test");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}