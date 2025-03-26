#include "behaviortree_ros2/bt_topic_pub_node_async.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace BT;
using Float32 = std_msgs::msg::Float32;

class PublishFloat : public AsyncRosTopicPubNode<Float32>
{

public:
  PublishFloat(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicPubNode<Float32>(name, conf, params) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<float>("data", 0., "Value of the float32 message")
    });
  }

  bool setMessage(Float32& msg) override
  {
    getInput("data", msg.data);
    return true;
  }
};