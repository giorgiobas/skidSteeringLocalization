#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class OdomNoiseNode : public rclcpp::Node
{
public:
    OdomNoiseNode()
        : Node("odom_noise_node"),
          current_odom_{}
    {
        // Publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Subscriber
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odomWithCovarianceWithoutNoise", 10,
            std::bind(&OdomNoiseNode::odom_callback, this, std::placeholders::_1));

        
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
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

        delta_x += 0;
        delta_theta += 0;

        // TODO
        // Random noise
        /*
        delta_y += noise_distribution_(generator_);
        delta_x += noise_distribution_(generator_);
        delta_theta += theta_noise_distribution_(generator_);
        */
        
        
        // TODO
        // Multiple Steps
        
        //double new_theta = 0.0;

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
        
        // Update current odometry
        current_odom_.pose.pose.position.x += delta_x;
        //current_odom_.pose.pose.position.y += delta_y;

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

        current_odom_.pose.covariance = msg->pose.covariance;
        current_odom_.twist.covariance = msg->twist.covariance;


        // Publish the updated odometry
        odom_publisher_->publish(current_odom_);

        // Store the current message as previous for the next callback
        previous_odom_ = *msg;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Odometry previous_odom_;
    std::default_random_engine generator_;
    std::normal_distribution<double> noise_distribution_;
    std::normal_distribution<double> theta_noise_distribution_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNoiseNode>());
    rclcpp::shutdown();
    return 0;
}

