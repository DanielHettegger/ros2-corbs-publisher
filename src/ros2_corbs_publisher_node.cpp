#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>

#include "rclcpp/rclcpp.hpp"
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

      _frame_id = this->declare_parameter("frame_id", "camera_frame");
      _data_path = this->declare_parameter("data_path", "./");

     _depth_filenames_stream = std::ifstream{_data_path + "depth.txt"};
     _color_filenames_stream = std::ifstream{_data_path + "rgb.txt"};

      if (!_depth_filenames_stream.is_open() || !_color_filenames_stream.is_open())
        throw std::runtime_error{"Data files could not be read.."};

      // Skip ahead first 8 elements
      std::string s;
      for(int i = 0; i < 8; i++){
        _depth_filenames_stream >> s;
        _color_filenames_stream >> s;
      }

      _timer = this->create_wall_timer(
        33ms, std::bind(&CoRBSPublisher::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _color_image_publisher;

    std::string _frame_id; 
    std::string _data_path;

    std::ifstream _depth_filenames_stream;
    std::ifstream _color_filenames_stream;

    void timer_callback()
    {
      std::string depth_filename, color_filename, depth_time_stamp, color_time_stamp;
      std::stringstream debug_ss;

      _depth_filenames_stream >> depth_time_stamp >> depth_filename;
      _color_filenames_stream >> color_time_stamp >> color_filename;

      if(depth_filename.empty()){
        RCLCPP_INFO(this->get_logger(), "END OF STREAM REACHED");
        return;
      }


      depth_filename = _data_path + depth_filename.replace(depth_filename.find("\\"),1,"/");
      color_filename = _data_path + color_filename.replace(color_filename.find("\\"),1,"/");

      debug_ss << "Reading images:" << std::endl; 
      debug_ss << depth_time_stamp << ": " << depth_filename << std::endl;
      debug_ss << color_time_stamp << ": " << color_filename;
      RCLCPP_DEBUG(this->get_logger(), debug_ss.str());

      cv::Mat_<float> depth_map;
      cv::Mat depth_map_reduced;
      cv::Mat_<cv::Vec3b> color_map;

      cv::imread(depth_filename, -1).convertTo(depth_map, CV_32FC1);
      if(depth_map.empty()){
        RCLCPP_INFO(this->get_logger(), "END OF STREAM REACHED");
        return;
      }
      color_map = cv::imread(color_filename);

      auto depth_image_message = std::make_unique<sensor_msgs::msg::Image>();
      auto color_image_message = std::make_unique<sensor_msgs::msg::Image>();

      convert_frame_to_message(depth_map, *depth_image_message);
      convert_frame_to_message(color_map, *color_image_message);

      add_timestamp_string_to_message(depth_time_stamp, *depth_image_message);
      add_timestamp_string_to_message(color_time_stamp, *color_image_message);
      
      _depth_image_publisher->publish(std::move(depth_image_message));
      _color_image_publisher->publish(std::move(color_image_message));
    }

    //From ROS2 image_tools cam2image [https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp]
    void convert_frame_to_message(
      const cv::Mat & frame, sensor_msgs::msg::Image & msg)
    {
      msg.is_bigendian = false;
      // copy cv information into ros message
      msg.height = frame.rows;
      msg.width = frame.cols;
      msg.encoding = mat_type2encoding(frame.type());
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      size_t size = frame.step * frame.rows;
      msg.data.resize(size);
      memcpy(&msg.data[0], frame.data, size);
      msg.header.frame_id = _frame_id;
    }

    //From ROS2 image_tools cam2image [https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp]
    std::string mat_type2encoding(int mat_type)
    {
      switch (mat_type) {
        case CV_8UC1:
          return "mono8";
        case CV_8UC3:
          return "bgr8";
        case CV_16SC1:
          return "mono16";
        case CV_8UC4:
          return "rgba8";
        case CV_32FC1:
          return "32FC1";
        default:
          throw std::runtime_error("Unsupported encoding type");
      }
    }

    void add_timestamp_string_to_message(
      std::string timestamp_string, sensor_msgs::msg::Image & msg)
    {
      auto split = timestamp_string.find('.');
      msg.header.stamp.sec = std::stoi(timestamp_string.substr(0,split));
      msg.header.stamp.nanosec = std::stoi(timestamp_string.substr(split+1,timestamp_string.length())) * 1000;
    }
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoRBSPublisher>());
    rclcpp::shutdown();
    return 0;
  }