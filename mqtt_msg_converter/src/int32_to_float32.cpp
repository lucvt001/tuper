#include "mqtt_msg_converter/int32_to_float32.hpp"
using namespace std;

Int32ToFloat32::Int32ToFloat32() : Node("int_to_float_bridge")
{
  string input_topic = this->declare_parameter("input_topic", "int32_json");
  string output_topic = this->declare_parameter("output_topic", "float32_out");

  float_pub_ = this->create_publisher<Float32>(output_topic, 10);

  int_sub_ = this->create_subscription<Int32>(
    input_topic, 10,
    [this](const Int32::SharedPtr msg) {
      auto out_msg = Float32();
      out_msg.data = static_cast<float>(msg->data);
      float_pub_->publish(out_msg);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Subscribing to %s and publishing to %s", input_topic.c_str(), output_topic.c_str());
}

// ---------- main function ----------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Int32ToFloat32>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
