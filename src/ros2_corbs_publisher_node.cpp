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
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class CoRBSPublisher : public rclcpp::Node
{
  public:
    CoRBSPublisher()
    : Node("ros2_corbs_publisher_node")
    {
      _depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
      _color_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("color_image", 10);
      _trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

      _frame_id = this->declare_parameter("frame_id", "camera_frame");
      _data_path = this->declare_parameter("data_path", "./");
      _trajectory_path = this->declare_parameter("trajectory_path", "./");

     _depth_filenames_stream = std::ifstream{_data_path + "depth.txt"};
     _color_filenames_stream = std::ifstream{_data_path + "rgb.txt"};
     _trajectory_stream = std::ifstream{_trajectory_path + "groundtruth.txt"};

      if (!_depth_filenames_stream.is_open() || !_color_filenames_stream.is_open())
        throw std::runtime_error{"Data files could not be read.."};

      if (!_trajectory_stream.is_open())
        throw std::runtime_error{"Trajectory file could not be read.."};

      std::string s;
      for(int i = 0; i < 8; i++){
        _depth_filenames_stream >> s;
        _color_filenames_stream >> s;
      }

      for(int i = 0; i < 16; i++){
        _trajectory_stream >> s;
        RCLCPP_INFO(this->get_logger(), s);
      }
      _timer = this->create_wall_timer(
        33ms, std::bind(&CoRBSPublisher::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _color_image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _trajectory_publisher;

    std::string _frame_id; 
    std::string _data_path;
    std::string _trajectory_path;

    std::ifstream _depth_filenames_stream;
    std::ifstream _color_filenames_stream;
    std::ifstream _trajectory_stream;

    /*bool _once = false;
    std::string depth_filename, color_filename, depth_time_stamp, color_time_stamp;*/


    void timer_callback()
    {
      //if(!_once){
      std::string depth_filename, color_filename, depth_time_stamp, color_time_stamp;
      std::string tx, ty, tz, qx, qy, qz, qw, pose_timestamp;
      std::stringstream debug_ss;

      _depth_filenames_stream >> depth_time_stamp >> depth_filename;
      _color_filenames_stream >> color_time_stamp >> color_filename;

      do {
        _trajectory_stream >> pose_timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      } while(stod(depth_time_stamp) > stod(pose_timestamp));

      if(depth_filename.empty()){
        RCLCPP_INFO(this->get_logger(), "END OF STREAM REACHED");
        return;
      }


      depth_filename = _data_path + depth_filename.replace(depth_filename.find("\\"),1,"/");
      color_filename = _data_path + color_filename.replace(color_filename.find("\\"),1,"/");

      debug_ss << "Reading images:" << std::endl; 
      debug_ss << depth_time_stamp << ": " << depth_filename << std::endl;
      debug_ss << color_time_stamp << ": " << color_filename << std::endl;
      debug_ss << pose_timestamp << ": " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
      //debug_ss << (stod(depth_time_stamp) > stod(pose_timestamp) ? "depth > pose" : "depth < pose") << " " << stod(depth_time_stamp) - stod(pose_timestamp);
      RCLCPP_DEBUG(this->get_logger(), debug_ss.str());

      //_once = true;}

      cv::Mat_<float> depth_map;
      cv::Mat_<cv::Vec3b> color_map;
      //cv::Mat_<cv::Vec3b> color_map_resized;

      depth_map = cv::imread(depth_filename, CV_LOAD_IMAGE_ANYDEPTH);
      if(depth_map.empty()){
        RCLCPP_INFO(this->get_logger(), "END OF STREAM REACHED");
        return;
      }

      depth_map *= 0.0002 * 1000;
      
      color_map = cv::imread(color_filename);

      //cv::resize(color_map, color_map_resized, cv::Size(depth_map.cols ,depth_map.rows), 0, 0, CV_INTER_LINEAR);

      /*cv::imshow("depth_map", depth_map);
      cv::imshow("color_map", color_map);
      cv::waitKey(1);*/

      auto depth_image_message = std::make_unique<sensor_msgs::msg::Image>();
      auto color_image_message = std::make_unique<sensor_msgs::msg::Image>();

      convert_frame_to_message(depth_map, *depth_image_message);
      convert_frame_to_message(color_map, *color_image_message);

      add_timestamp_string_to_message(depth_time_stamp, *depth_image_message);
      add_timestamp_string_to_message(color_time_stamp, *color_image_message);
      
      _depth_image_publisher->publish(std::move(depth_image_message));
      _color_image_publisher->publish(std::move(color_image_message));


      // Publish current pose
      auto pose_message = std::make_unique<geometry_msgs::msg::PoseStamped>();
      add_timestamp_string_to_message(pose_timestamp, *pose_message);

      pose_message->pose.position.x = stod(tx);
      pose_message->pose.position.y = stod(ty);
      pose_message->pose.position.z = stod(tz);
      pose_message->pose.orientation.x = stod(qx);
      pose_message->pose.orientation.y = stod(qy);
      pose_message->pose.orientation.z = stod(qz);
      pose_message->pose.orientation.w = stod(qw);

      _trajectory_publisher->publish(std::move(pose_message));

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

    void add_timestamp_string_to_message(
      std::string timestamp_string, geometry_msgs::msg::PoseStamped& msg)
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