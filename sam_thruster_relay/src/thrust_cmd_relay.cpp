#include "sam_thruster_relay/thrust_cmd_relay.hpp"

ThrustCmdRelay::ThrustCmdRelay() : Node("thrust_cmd_relay")
{
  string input_topic = this->declare_parameter<string>("input_topic", "");
  string output_topic1 = this->declare_parameter<string>("output_topic1", "");
  string output_topic2 = this->declare_parameter<string>("output_topic2", "");
  
  rpm_pub1_ = this->create_publisher<ThrusterRPM>(output_topic1, 2);
  rpm_pub2_ = this->create_publisher<ThrusterRPM>(output_topic2, 2);
  
  cmd_float_sub_ = this->create_subscription<Float32>(
    input_topic, 10, [this](const Float32::SharedPtr msg) 
    { 
      float rpm_float = msg->data; 
      publishRPM(rpm_float);
    });


  // timer_ = this->create_wall_timer(
  //   100ms, std::bind(&ThrustCmdRelay::publishRPM, this));   // 10 Hz
}

void ThrustCmdRelay::publishRPM(float rpm_float)
{
  auto message = ThrusterRPM();
  message.rpm = static_cast<int>(rpm_float * 1000);
  rpm_pub1_->publish(message);
  rpm_pub2_->publish(message);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrustCmdRelay>());
  rclcpp::shutdown();
  return 0;
}