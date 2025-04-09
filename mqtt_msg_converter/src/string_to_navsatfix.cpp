#include "mqtt_msg_converter/string_to_navsatfix.hpp"

#include <nlohmann/json.hpp>  // JSON parsing library
using json = nlohmann::json;
using namespace std;

StringToNavSatFix::StringToNavSatFix() : Node("navsatfix_json_bridge")
{
  string input_topic = this->declare_parameter("input_topic", "navsatfix_json");
  string output_topic = this->declare_parameter("output_topic", "navsatfix_out");
  json_sub_ = this->create_subscription<String>(
    input_topic, 2,
    std::bind(&StringToNavSatFix::jsonCallback, this, std::placeholders::_1)
  );

  navsatfix_pub_ = this->create_publisher<NavSatFix>(output_topic, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribing to %s and publishing to %s", input_topic.c_str(), output_topic.c_str());
}

void StringToNavSatFix::jsonCallback(const String::SharedPtr msg)
{
  try {
    json j = json::parse(msg->data);

    double lat = j.at("latitude").get<double>();
    double lon = j.at("longitude").get<double>();
    double alt = j.at("altitude").get<double>();

    auto navsat_msg = NavSatFix();
    navsat_msg.latitude = lat;
    navsat_msg.longitude = lon;
    navsat_msg.altitude = alt;

    // You can optionally set frame_id or stamp here
    navsat_msg.header.stamp = this->now();
    navsat_msg.header.frame_id = "gps";

    navsatfix_pub_->publish(navsat_msg);
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse JSON: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StringToNavSatFix>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
