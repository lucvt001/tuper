#include "behaviortree_ros2/bt_topic_sub_node_async.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace BT;
using namespace std;
using Float32 = std_msgs::msg::Float32;

class GetFloatFromTopic : public AsyncRosTopicSubNode<Float32>
{
public:
  GetFloatFromTopic(const string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicSubNode<Float32>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({
      OutputPort<float>("data", "Data received from the topic (float)")
    });
  }

  NodeStatus onTick(const std::shared_ptr<Float32>& msg) override
  {
    if (!msg)
    {
      return NodeStatus::RUNNING;
    }

    setOutput("data", msg->data);
    return NodeStatus::SUCCESS;
  }

  bool latchLastMessage() const override
  {
    return true;
  }
};