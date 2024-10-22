/*
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class GroundTruthPublisher : public rclcpp::Node {
public:
    GroundTruthPublisher() : Node("groundTruthPublisher") {
        // Create a TransformBuffer and a TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a publisher for the groundTruth topic
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("groundTruth", 10);

        // Timer to periodically publish pose data
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GroundTruthPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Get the transform from odom to base_link
            transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            publish_pose(transform);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    void publish_pose(const geometry_msgs::msg::TransformStamped &transform) {
        geometry_msgs::msg::PoseStamped pose_msg;

        // Fill in the header
        pose_msg.header.stamp = this->now();  // Current time
        pose_msg.header.frame_id = "odom";    // Parent frame

        // Fill in the position
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;

        // Fill in the orientation
        pose_msg.pose.orientation = transform.transform.rotation;

        // Publish the PoseStamped message
        pose_pub_->publish(pose_msg);
        // RCLCPP_INFO(this->get_logger(), "Published groundTruth pose data");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class GroundTruthPublisher : public rclcpp::Node {
public:
    GroundTruthPublisher() : Node("groundTruthPublisher") {
        // Create a TransformBuffer and a TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a publisher for the groundTruth topic
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("groundTruth", 10);

        // Timer to periodically publish pose data
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GroundTruthPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Get the transform from odom to base_link
            transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            store_and_publish_pose(transform);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    void store_and_publish_pose(const geometry_msgs::msg::TransformStamped &transform) {
        // Create a Pose and fill in the position and orientation
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;

        // Store the pose in the array
        pose_array_msg_.poses.push_back(pose);
        pose_array_msg_.header.stamp = this->now();  // Current time
        pose_array_msg_.header.frame_id = "odom";    // Parent frame

        // Publish the PoseArray message
        pose_pub_->publish(pose_array_msg_);
        // RCLCPP_INFO(this->get_logger(), "Published groundTruth pose array data");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseArray pose_array_msg_;  // Store all poses
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}


