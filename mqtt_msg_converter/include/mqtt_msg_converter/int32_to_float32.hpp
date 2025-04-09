#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;

class Int32ToFloat32 : public rclcpp::Node
{
public:
  Int32ToFloat32();

private:
  rclcpp::Publisher<Float32>::SharedPtr float_pub_;
  rclcpp::Subscription<Int32>::SharedPtr int_sub_;
};
