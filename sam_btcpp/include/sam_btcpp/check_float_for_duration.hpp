#include "behaviortree_ros2/bt_topic_sub_node_async.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std;
using namespace BT;
using Float32 = std_msgs::msg::Float32;

class CheckFloatForDuration : public AsyncRosTopicSubNode<Float32>
{
public:
  CheckFloatForDuration(const string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicSubNode<Float32>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<float>("min"),
      InputPort<float>("max"),
      InputPort<float>("min_success_duration", 1.0, "Minimum duration to consider the condition successful"),
      InputPort<float>("min_failure_duration", 1.0, "Minimum duration to consider the condition failed")
    });
  }

  NodeStatus onTick(const std::shared_ptr<Float32>& last_msg) override
  {
    float min = getInput<float>("min").value();
    float max = getInput<float>("max").value();
    if (min > max) std::swap(min, max);
    if (abs(min - max) < 1e-5) throw std::runtime_error("min and max values are the same");
    float min_success_duration = getInput<float>("min_success_duration").value();
    float min_failure_duration = getInput<float>("min_failure_duration").value();

    double time_now = now();

    if (abs(last_ticked_time_) < 1e-5)  // first tick
      last_ticked_time_ = time_now;

    if ((time_now - last_ticked_time_) > 1.0) // more than 1 second since last tick, meaning the node is being reused at a differnt time
    {
      last_success_time_ = 0.0;
      last_failure_time_ = 0.0;
      status_ = NodeStatus::RUNNING;
      // cout << "Resetting timers" << endl;
    }
    last_ticked_time_ = time_now;

    if(last_msg)  // empty if no new message received, since the last tick
    {
      float value = last_msg->data;
      bool condition = value >= min && value <= max;
      
      if (condition)
      {
        last_failure_time_ = 0.0;
        // ... if the condition is correct for the first time -> start success timer
        if (abs(last_success_time_) < 1e-5)
        {
          last_success_time_ = time_now;
        }
        // ... if the condition is continuously correct -> Set status_ to SUCCESS, stop timers
        else if ((time_now - last_success_time_) > min_success_duration)
        {
          status_ = NodeStatus::SUCCESS;
          last_success_time_ = 0.0;
        }
      } else 
      {
        last_success_time_ = 0.0;
        if (last_failure_time_ == 0.0)
        {
          last_failure_time_ = time_now;
        } else if ((time_now - last_failure_time_) > min_failure_duration)
        {
          status_ = NodeStatus::FAILURE;
          last_failure_time_ = 0.0;
        }
      }
    } else 
    {
      if (abs(last_failure_time_) < 1e-5)
      {
        last_failure_time_ = time_now;
      } else if ((time_now - last_failure_time_) > min_failure_duration)
      {
        status_ = NodeStatus::FAILURE;
        last_success_time_ = 0.0;
        last_failure_time_ = 0.0;
      }
    }
    return status_;
  }

  // virtual void halt() override
  // {
  //   AsyncRosTopicSubNode<Float32>::halt();
  //   // cout << "Resetting timers" << endl;
  //   // last_success_time_ = 0.0;
  //   // last_failure_time_ = 0.0;
  //   status_ = NodeStatus::RUNNING;
  // }

private:
  double last_success_time_ = 0.;
  double last_failure_time_ = 0.;
  double last_ticked_time_ = 0.;
  NodeStatus status_ = NodeStatus::RUNNING;
};