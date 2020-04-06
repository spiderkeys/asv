#include <functional>
#include <csignal>
#include <thread>
#include <string>

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

// ZED
#include <sl/Camera.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// Local
#include "recorder/periodic_scheduler.hpp"

class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode();
  virtual ~RecorderNode();
  
  void update();

private:
  // Constants
  const std::string TOPIC_CAMERA_LEFT     = "camera_left/image_raw/compressed";
  const std::string TOPIC_CAMERA_RIGHT    = "camera_right/image_raw/compressed";
  const std::string TOPIC_CAMERA_DEPTH    = "camera_depth/image_raw/compressed";
  const std::string TOPIC_ENABLED         = "enabled";

  const std::string TOPIC_TIME            = "time_unix_ms";
  const std::string TOPIC_GPS             = "gps";
  const std::string TOPIC_ATTITUDE        = "attitude";
  const std::string TOPIC_SPEED           = "speed"; 
  const std::string TOPIC_DEPTH           = "depth";
  const std::string TOPIC_ENABLE          = "enable";

  // Attributes
  std::string                           _data_dir;
  std::chrono::steady_clock::time_point _last_pub_time;
  
  // State
  uint64_t  _time           = 0;
  bool      _is_enabled     = false;
  bool      _got_gps        = false;
  bool      _got_time       = false;
  int64_t   _frame_number   = 0;

  double    _lat            = 0.0;
  double    _long           = 0.0;  
  double    _speed          = 0.0;
  double    _depth          = 0.0;
  double    _depth_conf     = 0.0;
  double    _roll           = 0.0;
  double    _pitch          = 0.0;
  double    _yaw            = 0.0;


  // ZED Camera
  sl::InitParameters  _cam_params;
  sl::Camera          _camera;
  sl::Mat             _image;
  sl::Mat             _depth_raw;
  sl::Mat             _depth_image;

  // Image Data
  std::unique_ptr<uint8_t[]> _data_left;
  std::unique_ptr<uint8_t[]> _data_right;
  std::unique_ptr<uint8_t[]> _data_depth_raw;
  std::unique_ptr<uint8_t[]> _data_depth_image;

  std::vector<uint8_t> _jpeg_buffer;

  // ROS2 Publishers & Subscribers
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr     _pub_camera_left;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr     _pub_camera_depth;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   _pub_enabled;

  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr              _sub_time;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        _sub_gps;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _sub_attitude;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _sub_speed;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr            _sub_depth;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                _sub_enable;

  // Update timer
  PeriodicScheduler _scheduler;

  void create_subscriptions();
  void write_data();
};
