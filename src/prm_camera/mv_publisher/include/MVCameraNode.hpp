#ifndef MV_CAMERA_NODE_HPP
#define MV_CAMERA_NODE_HPP

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "CameraApi.h"

class MVCameraNode : public rclcpp::Node
{
public:
    explicit MVCameraNode(const rclcpp::NodeOptions &);
    ~MVCameraNode() override;

private:
    void capture_frame();
    void declareParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &);

    int h_camera_;
    uint8_t *pby_buffer_;
    tSdkCameraCapbility t_capability_; // 设备描述信息
    tSdkFrameHead s_frame_info_;       // 图像帧头信息

    sensor_msgs::msg::Image image_msg_;

    image_transport::CameraPublisher camera_pub_;

    // RGB Gain
    int r_gain_, g_gain_, b_gain_;

    bool flip_image_;

    int fps_;
    bool fps_changed_;

    uint32_t seq_;

    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    std::thread capture_thread_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // Custom: Stop camera if navigating
    std::string nav_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;

};

#endif // MV_CAMERA_NODE_HPP