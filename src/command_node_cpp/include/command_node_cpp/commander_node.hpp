#include "geometry_msgs/msg/vector3.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/timer.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CommanderNode : public rclcpp::Node
{
    // rclcpp::Subscription<typename MessageT>
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;
public:
    CommanderNode() : rclcpp::Node("commander")
    {
        _pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        _timer = this->create_wall_timer(500ms, [this] {
            geometry_msgs::msg::Twist msg;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.01;
            msg.linear.x = 0.05;
            msg.linear.y = 0;
            msg.linear.z = 0;
            RCLCPP_INFO(this->get_logger(), "Publishing twist\n");
            _pub->publish(msg);
        });
    }
};
