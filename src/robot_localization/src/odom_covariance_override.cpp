#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

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
        publisherOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        //publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // IMU
        // Subscribe to the /imuWithoutCovariance topic
        subscriptionImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imuWithoutCovariance", 10,
            std::bind(&OdomCovarianceOverrideNode::imu_callback, this, std::placeholders::_1));

        // Publisher for the new /imu topic with covariance
        publisherImu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

         // Subscriber to GPS raw data
        gps_raw_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/raw", 10,
            std::bind(&OdomCovarianceOverrideNode::gpsCallback, this, std::placeholders::_1));

        // Subscriber to robot position
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odomGroundTruth", 10,
            std::bind(&OdomCovarianceOverrideNode::odomCallback, this, std::placeholders::_1));

        // Publisher for filtered GPS data
        gps_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    }

private:
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // ODOMETRY
        if (previous_odom_.header.stamp.sec == 0 && previous_odom_.header.stamp.nanosec == 0) {
            // If there's no previous odometry, just store the current and return
            previous_odom_ = *msg;
            current_odom_ = *msg; // Initialize current odometry
            return;
        }

        // Calculate the difference
        double delta_x = msg->pose.pose.position.x - previous_odom_.pose.pose.position.x;
        double delta_y = msg->pose.pose.position.y - previous_odom_.pose.pose.position.y;

        // Calculate theta from the quaternion
        tf2::Quaternion current_orientation, previous_orientation;
        tf2::fromMsg(msg->pose.pose.orientation, current_orientation);
        tf2::fromMsg(previous_odom_.pose.pose.orientation, previous_orientation);

        double theta_current = tf2::getYaw(current_orientation);
        double theta_previous = tf2::getYaw(previous_orientation);
        double delta_theta = theta_current - theta_previous;

        // Add noise to the difference
            // Random number generator
        generator_ = std::default_random_engine(std::random_device{}());
        double totalDistance = sqrt(delta_x*delta_x + delta_y*delta_y);
        noise_distribution_ = std::normal_distribution<double>(0.0, totalDistance/2); // Mean, Stddev //0.1
        theta_noise_distribution_ = std::normal_distribution<double>(0.0, delta_theta/3); // Mean, Stddev //0.05
        velocity_noise_distribution_ = std::normal_distribution<double>(0.0, 0.05);

        delta_x += 0;
        delta_theta += 0;
        current_odom_.twist = msg->twist;

        // TODO
        // Random noise
        /*
        delta_y += noise_distribution_(generator_);
        delta_x += noise_distribution_(generator_);
        delta_theta += theta_noise_distribution_(generator_);
        current_odom_.twist.twist.linear.x += velocity_noise_distribution_(generator_);
        current_odom_.twist.twist.angular.z += velocity_noise_distribution_(generator_);
        */
        
        // TODO
        // Multiple Steps
        
        //double new_theta = 0.0;
        /*
        current_odom_.pose.pose.position.y = 0;
        if(current_odom_.pose.pose.position.x>1){
            current_odom_.pose.pose.position.y = 0.5;
            //new_theta = 0.7;
        }
        if(current_odom_.pose.pose.position.x>2){
            current_odom_.pose.pose.position.y = -0.5;
            //new_theta = -0.7;
        }
        if(current_odom_.pose.pose.position.x>3){
            current_odom_.pose.pose.position.y = 0.5;
            //new_theta = 0.7;
        }
        if(current_odom_.pose.pose.position.x>4){
            current_odom_.pose.pose.position.y = -0.5;
            //new_theta = -0.7;
        }
        */
        
        // Update current odometry
        current_odom_.pose.pose.position.x += delta_x;
        current_odom_.pose.pose.position.y += delta_y;

        //TODO
        //current_odom_.pose.pose.position.y = current_odom_.pose.pose.position.x;

        // Update orientation based on the new theta
        double new_theta = theta_previous + delta_theta;
        
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, 0, new_theta);
        tf2_quat.normalize();
        // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

        current_odom_.pose.pose.orientation = msg_quat;

        // Copy the current timestamp and frame ID
        current_odom_.header.stamp = msg->header.stamp;
        current_odom_.header.frame_id = msg->header.frame_id;


        // COVARIANCE
        // Override the pose covariance
        current_odom_.pose.covariance = msg->pose.covariance;
        current_odom_.pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 0.1
        };

        // Override the twist covariance
        current_odom_.twist.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 0.2
        };

        // Store the current message as previous for the next callback
        previous_odom_ = *msg;

        // Publish the modified odometry message
        publisherOdom_->publish(current_odom_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Override the orientation covariance
        msg->orientation_covariance = {
            0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1
        };

        // This part is already taken care of in the .sdf 
        /*
        // Override the angular velocity covariance
        msg->angular_velocity_covariance = {
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        };

        // Override the linear acceleration covariance
        msg->linear_acceleration_covariance = {
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        };
        */

        // Publish the modified IMU message
        publisherImu_->publish(*msg);
    }

    // Callback to update the robot's position
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    // Callback to process GPS data
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Republish GPS data
        gps_fix_pub_->publish(*msg);
            
        // Check if the robot is outside the square
        /*
        if (isOutsideRestrictedArea(robot_x_, robot_y_))
        {
            // Republish GPS data
            gps_fix_pub_->publish(*msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GPS signal lost: robot is within the restricted area.");
        }
        */
    }

    // Helper function to check if the robot is outside the restricted square
    // Turtlebot
    /*
    bool isOutsideRestrictedArea(double x, double y)
    {
        return !(x >= -1.0 && x <= 2.0 && y >= 1.0 && y <= 4.0);
    }
    */

    // Scout Mini
    bool isOutsideRestrictedArea(double x, double y)
    {
        return !(x >= 2.0 && x <= 4.0 && y >= 0.0 && y <= 2.0);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriptionImu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherImu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriptionOdom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherOdom_;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Odometry previous_odom_;
    std::default_random_engine generator_;
    std::normal_distribution<double> noise_distribution_;
    std::normal_distribution<double> theta_noise_distribution_;
    std::normal_distribution<double> velocity_noise_distribution_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_raw_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomCovarianceOverrideNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
