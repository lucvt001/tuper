#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/controls/reactive_parallel_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sam_btcpp/get_ros_time.hpp"
#include "sam_btcpp/publish_rpm.hpp"
#include "sam_btcpp/publish_thrust_vector.hpp"
#include "sam_btcpp/publish_float.hpp"
#include "sam_btcpp/check_float_for_duration.hpp"
#include "sam_btcpp/get_float_from_topic.hpp"
#include "sam_btcpp/run_pid_client.hpp"
#include "sam_btcpp/log_to_ros.hpp"
#include "sam_btcpp/utils.hpp"

using namespace BT;
using namespace std;

// headers for signal handling
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

volatile sig_atomic_t stop;
void inthand(int signum) {
  (void)signum;
  stop = 0;
}

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("sam_btcpp");

  RosNodeParams params;
  params.nh = nh;
  params.wait_for_server_timeout = std::chrono::milliseconds(2000);

  BehaviorTreeFactory factory;

  factory.registerNodeType<ReactiveParallelNode>("ReactiveParallel");
  factory.registerNodeType<GetRosTime>("GetRosTime", params);
  factory.registerNodeType<PublishRPM>("PublishRPM", params);
  factory.registerNodeType<PublishThrustVector>("PublishThrustVector", params);
  factory.registerNodeType<PublishFloat>("PublishFloat", params);
  factory.registerNodeType<CheckFloatForDuration>("CheckFloatForDuration", params);
  factory.registerNodeType<GetFloatFromTopic>("GetFloatFromTopic", params);
  factory.registerNodeType<RunPidClient>("RunPid", params);
  factory.registerNodeType<LogToRos>("LogToRos", params);

  // Register all the XML files in the specified directory
  nh->declare_parameter("xml_directory", "");
  string xml_directory = nh->get_parameter("xml_directory").as_string();
  using filesystem::directory_iterator;
  for (auto const& entry : directory_iterator(xml_directory)) 
  {
    if( entry.path().extension() == ".xml")
      factory.registerBehaviorTreeFromFile(entry.path().string());
  }

  // Create the tree
  nh->declare_parameter("tree_name", "");
  string tree_name = nh->get_parameter("tree_name").as_string();
  auto tree = factory.createTree(tree_name);

  // Log to a randomly generated btlog file
  nh->declare_parameter("btlog_output_folder", "");
  string log_folder = nh->get_parameter("btlog_output_folder").as_string();
  std::unique_ptr<FileLogger2> logger;
  if (log_folder != "") {
    string log_file = generateBTLogFileName();
    log_file = log_folder + log_file;
    logger = std::make_unique<FileLogger2>(tree, log_file);
    RCLCPP_INFO(nh->get_logger(), "Logging to %s", log_file.c_str());
  }

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  nh->declare_parameter("do_connect_groot2", false);
  bool do_connect_groot2 = nh->get_parameter("do_connect_groot2").as_bool();
  std::unique_ptr<Groot2Publisher> publisher;
  if (do_connect_groot2)
  {
    const unsigned port = 1667;
    publisher = std::make_unique<Groot2Publisher>(tree, port);
    RCLCPP_INFO(nh->get_logger(), "Connecting to Groot2 on port %d", port);
  }

  NodeStatus status = tree.tickOnce();

  signal(SIGINT, inthand);

  nh->declare_parameter("loop_rate", 10);
  int loop_rate = nh->get_parameter("loop_rate").as_int();
  int sleep_time = static_cast<int>(1000/loop_rate);
  while( !stop && rclcpp::ok() && (status == NodeStatus::RUNNING)) 
  {  
    status = tree.tickOnce();
    tree.sleep(chrono::milliseconds(sleep_time));
  }

  return 0;
}