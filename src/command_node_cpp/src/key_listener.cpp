#include "command_node_cpp/keyboard_listener.hpp"
#include <iostream>


void KeyListen(std::function<void(char c)> f)
{
    while (1) {
        char k = std::cin.get();
        f(k);
    }
}
