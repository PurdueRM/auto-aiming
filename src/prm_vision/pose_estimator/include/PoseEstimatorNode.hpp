#ifndef POSE_ESTIMATOR_NODE_HPP
#define POSE_ESTIMATOR_NODE_HPP

// OpenCV and ROS2 includes
#include <opencv2/opencv.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"
#include "vision_msgs/msg/key_points.hpp"
#include "vision_msgs/msg/predicted_armor.hpp"

// tf publishing
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Pose Estimator classes
#include "PoseEstimator.h"
#include "ValidityFilter.hpp"

class PoseEstimatorNode : public rclcpp::Node
{
public:
    PoseEstimatorNode(const rclcpp::NodeOptions &options);
    ~PoseEstimatorNode();

private:
    PoseEstimator *pose_estimator;
    double _last_yaw_estimate = 0.0;
    int _allowed_missed_frames_before_no_fire = 0; // Gets set to 5 when we have a valid pose estimate

    // Class methods
    void publishZeroPredictedArmor(std_msgs::msg::Header header);

    // Callbacks and publishers/subscribers
    rclcpp::Subscription<vision_msgs::msg::KeyPoints>::SharedPtr key_points_subscriber;
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::PredictedArmor>> predicted_armor_publisher;
    void keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr msg);
};

#endif // POSE_ESTIMATOR_NODE_HPP