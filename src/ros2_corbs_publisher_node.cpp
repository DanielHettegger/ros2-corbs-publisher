#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;


class CoRBSPublisher : public rclcpp::Node
{
  public:
    CoRBSPublisher()
    : Node("ros2_corbs_publisher_node")
    {
      _depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
      _color_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
      _timer = this->create_wall_timer(
        33ms, std::bind(&CoRBSPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto depth_image_message = sensor_msgs::msg::Image();
      auto color_image_message = sensor_msgs::msg::Image();
      _depth_image_publisher->publish(depth_image_message);
      _color_image_publisher->publish(color_image_message);
    }
    
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::Image>::SharedPtr _depth_image_publisher;
    rclcpp::Publisher<std_msgs::msg::Image>::SharedPtr _color_image_publisher;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoRBSPublisher>());
    rclcpp::shutdown();
    return 0;
  }