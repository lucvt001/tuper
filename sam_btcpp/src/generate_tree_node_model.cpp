#include "behaviortree_cpp/xml_parsing.h"
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

// headers for writing and file operation
#include <iostream>
#include <fstream>

using namespace BT;

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("generate_tree_node_model");

  BehaviorTreeFactory factory;

  factory.registerNodeType<ReactiveParallelNode>("ReactiveParallel");
  factory.registerNodeType<GetRosTime>("GetRosTime", RosNodeParams(nh));
  factory.registerNodeType<PublishRPM>("PublishRPM", RosNodeParams(nh));
  factory.registerNodeType<PublishThrustVector>("PublishThrustVector", RosNodeParams(nh));
  factory.registerNodeType<PublishFloat>("PublishFloat", RosNodeParams(nh));
  factory.registerNodeType<CheckFloatForDuration>("CheckFloatForDuration", RosNodeParams(nh));
  factory.registerNodeType<GetFloatFromTopic>("GetFloatFromTopic", RosNodeParams(nh));
  factory.registerNodeType<RunPidClient>("RunPid", RosNodeParams(nh));
  factory.registerNodeType<LogToRos>("LogToRos", RosNodeParams(nh));

  // Modify your xml_file_path here
  nh->declare_parameter("tree_node_model_xmlfile_path", "");
  std::string xml_file_path = nh->get_parameter("tree_node_model_xmlfile_path").as_string();
  std::cout << "Writing to " << xml_file_path << std::endl;

  std::ofstream out(xml_file_path);
  out << writeTreeNodesModelXML(factory);
  out.close();
}