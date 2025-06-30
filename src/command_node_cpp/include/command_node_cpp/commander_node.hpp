#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CommanderNode : public rclcpp::Node
{
    // rclcpp::Subscription<typename MessageT>
    rclcpp::Publisher<geometry_msgs::msg::Twist> _pub;
public:
    CommanderNode() : rclcpp::Node("commander")
    {
    _pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/vel");
    }
};
