#include "OpenCVArmorDetectorNode.hpp"

OpenCVArmorDetectorNode::OpenCVArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("opencv_armor_detector", options)
{
  RCLCPP_INFO(get_logger(), "OpenCVArmorDetectorNode has been started.");

  // These are dynamic ROS2 parameters, meaning they can be changed during runtime for testing purposes.
  _hue_range_limit = this->declare_parameter("_hue_range_limit", 30);
  _saturation_lower_limit = this->declare_parameter("_saturation_lower_limit", 100);
  _value_lower_limit = this->declare_parameter("_value_lower_limit", 150);
  _target_color = this->declare_parameter("_target_red", true) ? RED : BLUE;
  _max_missed_frames = this->declare_parameter("_max_missed_frames", 1);
  _reduce_search_area = this->declare_parameter("_reduce_search_area", true);

  // Callbacks and pub/sub
  params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&OpenCVArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
  keypoints_publisher = this->create_publisher<vision_msgs::msg::KeyPoints>("key_points", 10);
  target_color_subscriber = this->create_subscription<std_msgs::msg::String>(
      "color_set", 1, std::bind(&OpenCVArmorDetectorNode::target_color_callback, this, std::placeholders::_1));

  // Initialize the detector
  DetectorConfig config = {_target_color, _hue_range_limit, _saturation_lower_limit, _value_lower_limit, _max_missed_frames, _reduce_search_area};
  detector = new OpenCVArmorDetector(config);
}

OpenCVArmorDetectorNode::~OpenCVArmorDetectorNode() { delete detector; }

void OpenCVArmorDetectorNode::imageTransportInitilization()
{
  image_transport::ImageTransport it(this->shared_from_this());
  image_subscriber =
      it.subscribe("image_raw", 1,
                   std::bind(&OpenCVArmorDetectorNode::imageCallback, this,
                             std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult OpenCVArmorDetectorNode::target_color_callback(const std_msgs::msg::String::SharedPtr color_msg)
{
  RCLCPP_INFO(get_logger(), "Target color changed to: %s|", color_msg->data.c_str());
  return parameters_callback({rclcpp::Parameter("_target_red", strcmp(color_msg->data.c_str(), "red") == 0)});
}

rcl_interfaces::msg::SetParametersResult OpenCVArmorDetectorNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == "_hue_range_limit")
    {
      _hue_range_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New hue range limit: %d", _hue_range_limit);
    }
    else if (parameter.get_name() == "_saturation_lower_limit")
    {
      _saturation_lower_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New saturation lower limit: %d",
                  _saturation_lower_limit);
    }
    else if (parameter.get_name() == "_value_lower_limit")
    {
      _value_lower_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New value lower limit: %d",
                  _value_lower_limit);
    }
    else if (parameter.get_name() == "_target_red")
    {
      _target_color = parameter.as_bool() ? RED : BLUE;
      RCLCPP_INFO(get_logger(), "New target color: %s",
                  _target_color ? "red" : "blue");
    }
    else if (parameter.get_name() == "_max_missed_frames")
    {
      _max_missed_frames = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New max missed frames: %d",
                  _max_missed_frames);
    }
    else if (parameter.get_name() == "_reduce_search_area")
    {
      _reduce_search_area = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "New reduce search area: %s",
                  _reduce_search_area ? "true" : "false");
    }
    else
    {
      result.successful = false;
      return result;
    }
  }

  // Update the detector's config
  detector->setConfig({_target_color, _hue_range_limit, _saturation_lower_limit, _value_lower_limit, _max_missed_frames, _reduce_search_area});
  return result;
}

void OpenCVArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
  cv::Mat frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;
  cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));

  // Call the detector's search method with both HSV ranges
  std::vector<_Float32> points = detector->search(frame);

  // Prep the message to be published
  vision_msgs::msg::KeyPoints keypoints_msg;

  std::array<float, 8> points_array;
  std::copy(points.begin(), points.end(), points_array.begin());
  float h = std::min(cv::norm(points.at(1) - points.at(3)), cv::norm(points.at(5) - points.at(7)));
  float w = cv::norm((points.at(0) + points.at(2)) / 2 - (points.at(4) + points.at(6)) / 2);

  keypoints_msg.header = image_msg->header;
  keypoints_msg.points = points_array;
  keypoints_msg.is_large_armor = (w / h) > 3.0; // 3.0 is the width ratio threshold before it is considered a large armor

  // Publish the message
  keypoints_publisher->publish(keypoints_msg);
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto opencv_armor_detector_node =
      std::make_shared<OpenCVArmorDetectorNode>(options);
  opencv_armor_detector_node->imageTransportInitilization();

  exec.add_node(opencv_armor_detector_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
