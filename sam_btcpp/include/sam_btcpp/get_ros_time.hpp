#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace BT;

class GetRosTime : public SyncActionNode
{
public:
  GetRosTime(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : SyncActionNode(name, config), node_(params.nh)
  {}

  // This Action simply write a value in the port "text"
  NodeStatus tick() override
  {
    auto now = node_->get_clock()->now();
    setOutput("time", now.seconds());
    return NodeStatus::SUCCESS;
  }

  // A node having ports MUST implement this STATIC method
  static PortsList providedPorts()
  {
    return { OutputPort<double>("time", "Time since start of ros launch (secs)") };
  }

private:
  rclcpp::Node::SharedPtr node_;
};