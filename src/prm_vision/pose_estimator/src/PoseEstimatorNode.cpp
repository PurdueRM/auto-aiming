#include "PoseEstimatorNode.hpp"

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions &options) : Node("pose_estimator", options)
{
    RCLCPP_INFO(get_logger(), "PoseEstimatorNode has been started.");

    // Callbacks and pub/sub
    key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>("key_points", 10, std::bind(&PoseEstimatorNode::keyPointsCallback, this, std::placeholders::_1));
    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);

    // Dynamic parameters
    // Get ros2 parameters
    cam_barrel_roll = this -> declare_parameter("cam_barrel_roll", 0.0);
    cam_barrel_pitch = this -> declare_parameter("cam_barrel_pitch", 0.0);
    cam_barrel_yaw = this -> declare_parameter("cam_barrel_yaw", 0.0);
    cam_barrel_x = this -> declare_parameter("cam_barrel_x", 80.0);
    cam_barrel_y = this -> declare_parameter("cam_barrel_y", 70.0);
    cam_barrel_z = this -> declare_parameter("cam_barrel_z", 80.0);
    pose_estimator->setAllowedMissedFramesBeforeNoFire(this->declare_parameter("_allowed_missed_frames_before_no_fire", 15));
    pose_estimator->setNumFramesToFireAfter(this->declare_parameter("_num_frames_to_fire_after", 3));
    validity_filter_.setLockInAfter(this->declare_parameter("_lock_in_after", 3));
    validity_filter_.setMaxDistance(this->declare_parameter("_max_distance", 10000));
    validity_filter_.setMinDistance(this->declare_parameter("_min_distance", 10));
    validity_filter_.setMaxShiftDistance(this->declare_parameter("_max_shift_distance", 150));
    validity_filter_.setPrevLen(this->declare_parameter("_prev_len", 5));
    validity_filter_.setMaxDt(this->declare_parameter("_max_dt", 2000.0));
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PoseEstimatorNode::parameters_callback, this, std::placeholders::_1));
}

PoseEstimatorNode::~PoseEstimatorNode() { delete pose_estimator; }

rcl_interfaces::msg::SetParametersResult PoseEstimatorNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters successfully updated.";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "_num_frames_to_fire_after" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setNumFramesToFireAfter(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_num_frames_to_fire_after' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_lock_in_after" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setLockInAfter(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_lock_in_after' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMaxDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_min_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMinDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_min_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_shift_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMaxShiftDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_shift_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_prev_len" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setPrevLen(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_prev_len' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            validity_filter_.setMaxDt(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_dt' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "_allowed_missed_frames_before_no_fire" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setAllowedMissedFramesBeforeNoFire(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_allowed_missed_frames_before_no_fire' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "cam_barrel_roll" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_roll = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_roll' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "cam_barrel_pitch" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_pitch = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_pitch' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "cam_barrel_yaw" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_yaw = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_yaw' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "cam_barrel_x" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_x = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_x' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "cam_barrel_y" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_y = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_y' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "cam_barrel_z" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            cam_barrel_z = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'cam_barrel_z' updated to: %f", param.as_double());
        }
        else
        {
            result.successful = false;
            result.reason = "Invalid parameter or type.";
            RCLCPP_WARN(this->get_logger(), "Failed to update parameter: %s", param.get_name().c_str());
        }
    }

    return result;
}

void PoseEstimatorNode::keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr key_points_msg)
{
    if (key_points_msg->points.size() != 8 || key_points_msg->points[4] == 0) // idx 4 is x-coord of top right corner, so if it's 0, we know there's no armor
    {
        // No armor detected
        publishZeroPredictedArmor(key_points_msg->header, "NO_ARMOR");
        return;
    }

    cv::Mat tvec, rvec;
    bool reset_kalman = false;
    std::string new_auto_aim_status;
    std::vector<cv::Point2f> image_points;

    // Convert the message to a vector of cv::Point2f
    image_points.push_back(cv::Point2f(key_points_msg->points[0], key_points_msg->points[1]));
    image_points.push_back(cv::Point2f(key_points_msg->points[2], key_points_msg->points[3]));
    image_points.push_back(cv::Point2f(key_points_msg->points[4], key_points_msg->points[5]));
    image_points.push_back(cv::Point2f(key_points_msg->points[6], key_points_msg->points[7]));

    // Compute armor's pose and validate it. We compute new yaw estimate based on the last yaw estimate.
    pose_estimator->estimateTranslation(image_points, key_points_msg->is_large_armor, tvec, rvec);
    _last_yaw_estimate = pose_estimator->estimateYaw(_last_yaw_estimate, image_points, tvec);
    bool valid_pose_estimate = pose_estimator->isValid(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), new_auto_aim_status, reset_kalman);
    
    // Set up transformation matrices
    Eigen::Matrix<double, 3, 3> r_roll;
    r_roll << 1, 0, 0,
          0, cos(cam_barrel_roll), -sin(cam_barrel_roll),
          0, sin(cam_barrel_roll), cos(cam_barrel_roll);

    Eigen::Matrix<double, 3, 3> r_pitch;
    r_pitch << cos(cam_barrel_pitch), 0, -sin(cam_barrel_pitch),
           0, 1, 0,
           sin(cam_barrel_pitch), 0, cos(cam_barrel_pitch);

    Eigen::Matrix<double, 3, 3> r_yaw;
    r_yaw << cos(cam_barrel_yaw), sin(cam_barrel_yaw), 0,
         -sin(cam_barrel_yaw), cos(cam_barrel_yaw), 0,
         0, 0, 1;

    Eigen::Matrix<double, 3, 3> r_mat = r_roll * r_pitch * r_yaw;

    Eigen::Matrix<double, 4, 4> transform_mat;
    transform_mat << r_mat(0, 0), r_mat(0, 1), r_mat(0, 2), cam_barrel_x,
                 r_mat(1, 0), r_mat(1, 1), r_mat(1, 2), cam_barrel_y,
                 r_mat(2, 0), r_mat(2, 1), r_mat(2, 2), cam_barrel_z,
                 0, 0, 0, 1;


    // Multiply cam -> target vector by transformation matrix to get barrel -> target vector
    Eigen::Vector4d cam_to_target = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), 1};
    Eigen::Vector4d barrel_to_target = transform_mat * cam_to_target;

    // Set tvec to contain the transformed xyz coordinates
    tvec.at<double>(0) = barrel_to_target(0);
    tvec.at<double>(1) = barrel_to_target(1);
    tvec.at<double>(2) = barrel_to_target(2);

    // Publish the predicted armor if the pose is valid (we are tracking or firing)
    if (valid_pose_estimate)
    {
        vision_msgs::msg::PredictedArmor predicted_armor_msg;
        predicted_armor_msg.header = key_points_msg->header;
        predicted_armor_msg.x = tvec.at<double>(0);
        predicted_armor_msg.y = tvec.at<double>(1);
        predicted_armor_msg.z = tvec.at<double>(2);
        predicted_armor_msg.pitch = 15;                             // Pitch is always 15
        predicted_armor_msg.yaw = 180 * _last_yaw_estimate / CV_PI; // Convert to degrees
        predicted_armor_msg.roll = 0;                               // Roll is always 0
        predicted_armor_msg.x_vel = 0;
        predicted_armor_msg.y_vel = 0; // TODO: compute yaw rate
        predicted_armor_msg.z_vel = 0;
        predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");
        predicted_armor_publisher->publish(predicted_armor_msg);
    }

    else if (new_auto_aim_status == "STOPPING")
    {
        publishZeroPredictedArmor(key_points_msg->header, new_auto_aim_status);
    }

    // Draw top-down view
#ifdef DEBUG
    drawTopDownViewGivenRotation(_last_yaw_estimate, tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
#endif
}

void PoseEstimatorNode::drawTopDownViewGivenRotation(double yaw, double X, double Y, double Z)
{
    // Draw a line rotated from the x-axis by yaw.
    cv::Mat top_down_view = cv::Mat::zeros(1280, 720, CV_8UC3);

    // Draw the target offset from the center of the image
    int target_x = Z;
    int target_y = X;
    cv::circle(top_down_view, cv::Point(target_x, target_y), 5, cv::Scalar(0, 255, 0), -1);

    cv::resize(top_down_view, top_down_view, cv::Size(100, 100));
    cv::imshow("Top Down View", top_down_view);
    cv::waitKey(1);
}

void PoseEstimatorNode::publishZeroPredictedArmor(std_msgs::msg::Header header, std::string new_auto_aim_status)
{
    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    predicted_armor_msg.header = header;
    predicted_armor_msg.x = 0;
    predicted_armor_msg.y = 0;
    predicted_armor_msg.z = 0;
    predicted_armor_msg.pitch = 0;
    predicted_armor_msg.yaw = 0;
    predicted_armor_msg.roll = 0;
    predicted_armor_msg.x_vel = 0;
    predicted_armor_msg.y_vel = 0;
    predicted_armor_msg.z_vel = 0;
    predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");

    predicted_armor_publisher->publish(predicted_armor_msg);
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto pose_estimator_node = std::make_shared<PoseEstimatorNode>(options);

    exec.add_node(pose_estimator_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
