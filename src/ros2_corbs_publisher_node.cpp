#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;


class CoRBSPublisher : public rclcpp::Node
{
  public:
    CoRBSPublisher()
    : Node("ros2_corbs_publisher_node"), count_(0)
    {
    }

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoRBSPublisher>());
    rclcpp::shutdown();
    return 0;
  }