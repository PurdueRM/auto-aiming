#include <CameraApi.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "MVCameraNode.hpp"

#define WIDTH 1280
#define HEIGHT 1024

MVCameraNode::MVCameraNode(const rclcpp::NodeOptions &options) : Node("mv_camera", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

    // 没有连接设备
    if (i_camera_counts == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        return;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

    // 初始化失败
    RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Init failed!");
        return;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera_, &t_capability_);

    // 直接使用vector的内存作为相机输出buffer
    image_msg_.data.reserve(WIDTH * HEIGHT * 3);

    // 设置手动曝光
    CameraSetAeState(h_camera_, false);


    // Declare camera parameters
    declareParameters();

    // 让SDK进入工作模式，开始接收来自相机发送的图像
    // 数据。如果当前相机是触发模式，则需要接收到
    // 触发帧以后才会更新图像。
    CameraPlay(h_camera_);

    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

    // Create camera publisher
    // rqt_image_view can't subscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "mv_camera");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
        "camera_info_url", "package://mv_publisher/config/mv_133_4_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url))
    {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    // Add callback to the set parameter event
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MVCameraNode::parametersCallback, this, std::placeholders::_1));

    // Subscribe to navigation status
    nav_status_ = "";
    nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_status", 1,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            nav_status_ = msg->data;
        });

    seq_ = 0;
    capture_thread_ = std::thread(std::bind(&MVCameraNode::capture_frame, this));
}

void MVCameraNode::capture_frame()
{
    static std::string prior_nav_status = "";
    RCLCPP_INFO_ONCE(this->get_logger(), "Starting capture threads");
    
    image_msg_.header.frame_id = "camera_frame";
    image_msg_.encoding = "rgb8";

    rclcpp::WallRate loop_rate(fps_ + 1);
    rclcpp::WallRate delay(10); // Sleep when navigating

    while (rclcpp::ok())
    {
        if (prior_nav_status != nav_status_)
        {
            if (nav_status_ == "NAVIGATING")
            {
                RCLCPP_INFO(this->get_logger(), "Camera stopped due to navigation status: %s", nav_status_.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Camera resumed, current navigation status: %s", nav_status_.c_str());
            }
            prior_nav_status = nav_status_;
        }
        if (nav_status_ == "NAVIGATING")
        {
            delay.sleep();
            continue;
        }
        if (CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(h_camera_, pby_buffer_, image_msg_.data.data(), &s_frame_info_);
            if (flip_image_)
            {
                CameraFlipFrameBuffer(image_msg_.data.data(), &s_frame_info_, 3);
            }
            camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
            image_msg_.height = s_frame_info_.iHeight;
            image_msg_.width = s_frame_info_.iWidth;
            image_msg_.step = s_frame_info_.iWidth * 3;
            image_msg_.data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight * 3);

            camera_pub_.publish(image_msg_, camera_info_msg_);

            CameraReleaseImageBuffer(h_camera_, pby_buffer_);
            RCLCPP_INFO_ONCE(this->get_logger(), "First Frame Published");
            if (seq_ % 1000 == 0)
                RCLCPP_INFO(this->get_logger(), "seq: %d", seq_);
            seq_++;
            loop_rate.sleep();
        }
    }
}

MVCameraNode::~MVCameraNode()
{
    if (capture_thread_.joinable())
    {
        capture_thread_.join();
    }

    CameraUnInit(h_camera_);

    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void MVCameraNode::declareParameters()
{
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time, CMOS exposure in us
    param_desc.description = "Exposure time in microseconds";
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
    param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
    double exposure_time = this->declare_parameter("exposure_time", 4081, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

    // Analog gain
    param_desc.description = "Analog gain";
    param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiAnalogGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiAnalogGainMax;
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain = this->declare_parameter("analog_gain", analog_gain, param_desc);
    CameraSetAnalogGain(h_camera_, analog_gain);
    RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

    // RGB Gain default value
    CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
    // R Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iRGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iRGainMax;
    r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
    // G Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iGGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iGGainMax;
    g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
    // B Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iBGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iBGainMax;
    b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
    // Set gain
    CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);

    // Saturation
    param_desc.description = "Saturation";
    param_desc.integer_range[0].from_value = t_capability_.sSaturationRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = this->declare_parameter("saturation", saturation, param_desc);
    CameraSetSaturation(h_camera_, saturation);
    RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

    // Gamma
    param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = this->declare_parameter("gamma", gamma, param_desc);
    CameraSetGamma(h_camera_, gamma);
    RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

    // Flip
    flip_image_ = this->declare_parameter("flip_image", false);

    // FPS
    param_desc.description = "FPS";
    param_desc.integer_range[0].from_value = 1;
    param_desc.integer_range[0].to_value = 245;
    fps_ = this->declare_parameter("fps", 245, param_desc);
    RCLCPP_INFO(this->get_logger(), "FPS = %d", fps_);
}

rcl_interfaces::msg::SetParametersResult MVCameraNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {
        if (param.get_name() == "exposure_time")
        {
            int status = CameraSetExposureTime(h_camera_, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "analog_gain")
        {
            int status = CameraSetAnalogGain(h_camera_, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set analog gain, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "rgb_gain.r")
        {
            r_gain_ = param.as_int();
            int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "rgb_gain.g")
        {
            g_gain_ = param.as_int();
            int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "rgb_gain.b")
        {
            b_gain_ = param.as_int();
            int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "saturation")
        {
            int status = CameraSetSaturation(h_camera_, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set saturation, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "gamma")
        {
            int gamma = param.as_int();
            int status = CameraSetGamma(h_camera_, gamma);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                result.successful = false;
                result.reason = "Failed to set Gamma, status = " + std::to_string(status);
            }
        }
        else if (param.get_name() == "flip_image")
        {
            flip_image_ = param.as_bool();
        }
        else
        {
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
    }
    return result;
}

int main(int argc, char *argv[])
{

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto video_capture_node = std::make_shared<MVCameraNode>(options);

    exec.add_node(video_capture_node);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
