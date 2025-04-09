#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using String = std_msgs::msg::String;
using NavSatFix = sensor_msgs::msg::NavSatFix;

class StringToNavSatFix : public rclcpp::Node
{
public:
  StringToNavSatFix();

private:
  void jsonCallback(const String::SharedPtr msg);

  rclcpp::Subscription<String>::SharedPtr json_sub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr navsatfix_pub_;
};
