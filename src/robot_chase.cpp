#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;


class RobotChaseNode : public rclcpp::Node {
public:
    RobotChaseNode() : Node("robot_chase_node") {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher for Twist messages
        rick_cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 1);

        // Timer to update the control loop
        timer = this->create_wall_timer(0.05s, std::bind(&RobotChaseNode::controlLoop, this));
    }

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rick_cmd_vel_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time now = this->get_clock()->now();
    double kp_yaw = 0.7;
    double kp_distance = 0.5;

    void controlLoop() {
        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("rick/base_link", "morty/base_link", tf2::TimePoint(), tf2::durationFromSec(1.0));

            double error_distance = calculateDistanceError(transform);
            double error_yaw = calculateYawError(transform);

            geometry_msgs::msg::Twist twist;
            if (error_distance<0.40){
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                rick_cmd_vel_publisher->publish(twist);
            }
            else{
                twist.linear.x = kp_distance * error_distance; //std::min(0.5,kp_distance * error_distance);
                twist.angular.z = kp_yaw * error_yaw;
                rick_cmd_vel_publisher->publish(twist);
            }

            RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f, Angular Velocity: %f", twist.linear.x, twist.angular.z);
            RCLCPP_INFO(this->get_logger(), "Linear Error: %f, Angular Error: %f", error_distance, error_yaw);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Exception: %s", ex.what());
        }
    }

    double calculateDistanceError(const geometry_msgs::msg::TransformStamped &transform) {
        return sqrt(transform.transform.translation.x * transform.transform.translation.x +
                    transform.transform.translation.y * transform.transform.translation.y);
    }

    double calculateYawError(const geometry_msgs::msg::TransformStamped &transform) {
        return atan2(transform.transform.translation.y, transform.transform.translation.x);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotChaseNode>());
    rclcpp::shutdown();
    return 0;
}





