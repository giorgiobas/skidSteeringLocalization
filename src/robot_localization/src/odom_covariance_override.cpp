#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class OdomCovarianceOverrideNode : public rclcpp::Node
{
public:
    OdomCovarianceOverrideNode() : Node("odom_covariance_override")
    {
        // ODOM
        // Subscribe to the /odomWithoutCovariance topic
        subscriptionOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odomWithoutCovariance", 10,
            std::bind(&OdomCovarianceOverrideNode::odom_callback, this, std::placeholders::_1));

        // Publisher for the new /odom topic with covariance
        publisherOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odomWithCovarianceWithoutNoise", 10);
        //publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // IMU
        // Subscribe to the /imuWithoutCovariance topic
        subscriptionImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imuWithoutCovariance", 10,
            std::bind(&OdomCovarianceOverrideNode::imu_callback, this, std::placeholders::_1));

        // Publisher for the new /imu topic with covariance
        publisherImu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Override the pose covariance
        msg->pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 0.1
        };

        // Override the twist covariance
        msg->twist.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 0.1
        };

        // Publish the modified odometry message
        publisherOdom_->publish(*msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Override the orientation covariance
        msg->orientation_covariance = {
            0.00025, 0, 0,
            0, 0.00025, 0,
            0, 0, 0.00025
        };

        // Override the angular velocity covariance
        msg->angular_velocity_covariance = {
            0.00025, 0, 0,
            0, 0.00025, 0,
            0, 0, 0.00025
        };

        // Override the linear acceleration covariance
        msg->linear_acceleration_covariance = {
            0.00025, 0, 0,
            0, 0.00025, 0,
            0, 0, 0.00025
        };

        // Publish the modified IMU message
        publisherImu_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriptionImu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherImu_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriptionOdom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherOdom_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomCovarianceOverrideNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
