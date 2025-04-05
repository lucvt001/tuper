#ifndef THRUST_CMD_RELAY_HPP
#define THRUST_CMD_RELAY_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "smarc_msgs/msg/thruster_rpm.hpp"

using namespace std;
using Float32 = std_msgs::msg::Float32;
using ThrusterRPM = smarc_msgs::msg::ThrusterRPM;

class ThrustCmdRelay : public rclcpp::Node
{
public:
  ThrustCmdRelay();

private:
  void publishRPM(float rpm_float);

  rclcpp::Subscription<Float32>::SharedPtr cmd_float_sub_;
  rclcpp::Publisher<ThrusterRPM>::SharedPtr rpm_pub1_;
  rclcpp::Publisher<ThrusterRPM>::SharedPtr rpm_pub2_;
  // rclcpp::TimerBase::SharedPtr timer_;

  // float rpm_float_;
};

#endif // THRUST_CMD_RELAY_HPP