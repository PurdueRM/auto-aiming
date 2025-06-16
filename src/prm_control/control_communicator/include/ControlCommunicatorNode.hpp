#ifndef CONTROL_COMMUNICATOR_NODE_HPP
#define CONTROL_COMMUNICATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/predicted_armor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vision_msgs/msg/yaw_pitch.hpp"
#include "messages.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <memory>
#include <functional>
#include <string>
#include <chrono>

#include "ControlCommunicator.h"

class ControlCommunicatorNode : public rclcpp::Node
{
public:
	ControlCommunicatorNode(const char *port);
	~ControlCommunicatorNode();

	ControlCommunicator *control_communicator = new ControlCommunicator();

	void auto_aim_handler(const std::shared_ptr<vision_msgs::msg::PredictedArmor> msg);
	void nav_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
	void read_uart();

private:
	void publish_static_tf(float x, float y, float z, float roll, float pitch, float yaw, const char *frame_id, const char *child_frame_id);
	void heart_beat_handler();
	void measure_auto_aim_performance(bool AIMING);

	uint32_t auto_aim_frame_id = 0;
	uint32_t nav_frame_id = 0;
	uint32_t heart_beat_frame_id = 0;

	// Read UART results
	float yaw_vel = 0;	 // rad/s (+ccw, -cw)
	float pitch_vel = 0; // rad/s
	float pitch = 0;	 // rad (+up, -down)?
	bool is_enemy_red = 0;
	bool is_match_running = 0;
	bool valid_read = false;
	std::string old_target_robot_color; 

	const char *port;

	bool is_connected;
	float aim_bullet_speed;
	int aim_stop_null_frame_count;

	rclcpp::TimerBase::SharedPtr heart_beat_timer;
	rclcpp::Subscription<vision_msgs::msg::PredictedArmor>::SharedPtr auto_aim_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_robot_color_publisher;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr match_status_publisher;
	rclcpp::TimerBase::SharedPtr uart_read_timer;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};
#endif // CONTROL_COMMUNICATOR_NODE_HPP