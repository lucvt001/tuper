#ifndef LOG_HPP
#define LOG_HPP

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std;
using namespace BT;

class LogToRos : public SyncActionNode
{
public:
  LogToRos(const string& name, const NodeConfig& config, const RosNodeParams& params)
    : SyncActionNode(name, config), node_(params.nh) 
  {}

  static PortsList providedPorts()
  {
    return { 
      InputPort<string>("log_msg"),
      InputPort<string>("log_level", "INFO", "Log level: DEBUG, INFO, WARN, ERROR, FATAL") 
    };
  }

  NodeStatus tick() override
  {
    string log_msg;
    string log_level;

    if (!getInput<string>("log_msg", log_msg))
    {
      throw RuntimeError("missing required input [log_msg]");
    }

    if (!getInput<string>("log_level", log_level))
    {
      throw RuntimeError("missing required input [log_level]");
    }

    if (log_level == "DEBUG") {
      RCLCPP_DEBUG(node_->get_logger(), "%s", log_msg.c_str());
    } else if (log_level == "INFO") {
      RCLCPP_INFO(node_->get_logger(), "%s", log_msg.c_str());
    } else if (log_level == "WARN") {
      RCLCPP_WARN(node_->get_logger(), "%s", log_msg.c_str());
    } else if (log_level == "ERROR") {
      RCLCPP_ERROR(node_->get_logger(), "%s", log_msg.c_str());
    } else if (log_level == "FATAL") {
      RCLCPP_FATAL(node_->get_logger(), "%s", log_msg.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown log level: %s", log_level.c_str());
    }

    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#endif // LOG_HPP