
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>
// #include <math.h>

class ScanLimitNode : public rclcpp::Node
{
public:
    explicit ScanLimitNode(const rclcpp::NodeOptions &options)
        : Node("scan_limit_node", options)
    {
        // Declare parameters
        scan_limit_min_ = this->declare_parameter("scan_min_angle", -3.1241390705108643 / 2);
        scan_limit_max_ = this->declare_parameter("scan_max_angle", 3.1415927410125732 / 2);

        // Create publisher
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // Create subscriber
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10, std::bind(&ScanLimitNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "scan_limit_node has been initialized");
    }

    ~ScanLimitNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down scan_limit_node");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First /scan_raw received.");
        // Create new message
        auto scan_new_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_new_msg->header = msg->header;

        int orignal_size = msg->ranges.size();

        scan_new_msg->angle_min = scan_limit_min_;
        scan_new_msg->angle_max = scan_limit_max_;
        scan_new_msg->angle_increment = msg->angle_increment;
        scan_new_msg->time_increment = msg->time_increment;
        scan_new_msg->scan_time = msg->scan_time;
        scan_new_msg->range_min = msg->range_min;
        scan_new_msg->range_max = msg->range_max;

        // Calculate number of points to copy
        int new_size = (scan_limit_max_ - scan_limit_min_) / msg->angle_increment + 1;

        // printf("original size: %d, new size: %d", orignal_size, new_size);

        scan_new_msg->ranges.resize(new_size);
        scan_new_msg->intensities.resize(new_size);

        int i_org = 0;
        int i_new = 0;
        while (i_org <= orignal_size)
        {
            if (i_org * msg->angle_increment + msg->angle_min > scan_limit_min_)
            {
                // printf("writing from %d to %d", i_org, i_new);
                scan_new_msg->ranges[i_new] = msg->ranges[i_org];
                scan_new_msg->intensities[i_new] = msg->intensities[i_org];
                i_new++;
                // printf("i_new: %d, i_org: %d\n", i_new, i_org);
            }
            if (i_new >= new_size)
            {
                // printf("Breaking\n");
                break;
            }

            // if(i_org * msg->angle_increment + msg->angle_min > scan_limit_min_ && i_org * msg->angle_increment < scan_limit_max_){
            //     // printf("writing from %d to %d", i_org, i_new);
            //     scan_new_msg->ranges[i_new] = msg->ranges[i_org];
            //     scan_new_msg->intensities[i_new] = msg->intensities[i_org];
            //     i_new++;
            // }
            i_org++;
        }

        // Publish message
        scan_publisher_->publish(*scan_new_msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "First /scan published.");
    }

private:
    float scan_limit_min_;
    float scan_limit_max_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

int main(int argc, char *argv[])
{

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto scan_limit_node = std::make_shared<ScanLimitNode>(options);

    exec.add_node(scan_limit_node);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
