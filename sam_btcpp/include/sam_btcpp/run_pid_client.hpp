#include "behaviortree_ros2/bt_action_node.hpp"
#include "tuper_interfaces/action/run_pid.hpp"

using namespace std;
using namespace BT;
using RunPid = tuper_interfaces::action::RunPid;

class RunPidClient : public RosActionNode<RunPid>
{
public:
  RunPidClient(const string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<RunPid>(name, conf, params) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<float>("setpoint", 0.0, "The desired setpoint")
    });
  }

  bool setGoal(Goal& goal) override
  {
    goal.setpoint = getInput<float>("setpoint").value();
    return true;
  }
  
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    // This function is not very important
    // Because the intended use of pid server is to never return a result
    // The pid server will keep running until it is interrupted
    // The checking of conditions will be done by BT
    (void)wr;
    return NodeStatus::SUCCESS;
  }
};