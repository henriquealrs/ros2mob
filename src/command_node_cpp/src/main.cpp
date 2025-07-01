#include "command_node_cpp/commander_node.hpp"
#include "command_node_cpp/keyboard_listener.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <utility>

namespace {

std::pair<double, double> ProcessKey(char c)
{
    double linear_acc = 0.0;
    double angular_acc = 0.0;
    switch (c)
    {
        case 'k':
            linear_acc += 0.1;
            break;
        case 's':
            linear_acc -= 0.1;
            break;
        case 'd':
            angular_acc -= 0.01;
            break;
        case 'a':
            angular_acc += 0.01;
            break;
        default:
            break;
    }
    return std::make_pair(linear_acc, angular_acc);
}

void StartKeyListener(CommanderNode::SharedPtr& _node)
{
    std::thread t([](auto node){
        KeyListen([node&](char c){
            auto [lin_acc, ang_acc] = ProcessKey(c);
            node->
        })
    });
}

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());

    KeyListen([](char key){
        switch (key)
        {
            case '
        }
    });

    rclcpp::shutdown();
    return 0;
}
