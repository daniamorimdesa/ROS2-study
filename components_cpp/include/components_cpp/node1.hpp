#ifndef COMPONENTS_CPP__NODE1_HPP_
#define COMPONENTS_CPP__NODE1_HPP_

#include "rclcpp/rclcpp.hpp"

class Node1 : public rclcpp::Node
{
public:
    Node1();

private:
    void callbackTimer1();
    void callbackTimer2();
    void callbackTimer3();

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;
};

#endif  // COMPONENTS_CPP__NODE1_HPP_
