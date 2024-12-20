#ifndef OPENCV_ARMOR_DETECTOR_NODE_HPP
#define OPENCV_ARMOR_DETECTOR_NODE_HPP

#include "OpenCVArmorDetector.h"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

#include "vision_msgs/msg/key_points.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include <unordered_map>

class OpenCVArmorDetectorNode : public rclcpp::Node
{
public:
    OpenCVArmorDetectorNode(const rclcpp::NodeOptions &options);
    ~OpenCVArmorDetectorNode();

    // Setup
    void imageTransportInitilization();

private:
    OpenCVArmorDetector *detector;

    // Dynamic parameters
    int _hue_range_limit;
    int _saturation_lower_limit;
    int _value_lower_limit;
    TargetColor _target_color;
    int _max_missed_frames;
    bool _reduce_search_area;

    // Callbacks and publishers/subscribers
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::KeyPoints>> keypoints_publisher = NULL;
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // Subscribers
    image_transport::Subscriber image_subscriber;
};

#endif // OPENCV_ARMOR_DETECTOR_NODE_HPP
