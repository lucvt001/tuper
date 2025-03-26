#include <behaviortree_ros2/bt_service_node.hpp>
#include <tuper_interfaces/srv/set_pid_gains.hpp>

using namespace std;
using namespace BT;
using SetPidGains = tuper_interfaces::srv::SetPidGains;

class SetPid : public RosServiceNode<SetPidGains>
{
public:
  SetPid(const string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosServiceNode<SetPidGains>(name, conf, params) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<float>("kp", 0.0, "The proportional gain"),
      InputPort<float>("ki", 0.0, "The integral gain"),
      InputPort<float>("kd", 0.0, "The derivative gain")
    });
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    request->kp = getInput<float>("kp").value();
    request->ki = getInput<float>("ki").value();
    request->kd = getInput<float>("kd").value();
    return true;
  }

  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    if(response->is_success)
      return NodeStatus::SUCCESS;
    else
      return NodeStatus::FAILURE;
  }
  
};