#ifndef FORMATION_SHAPE_BROADCASTER_HPP
#define FORMATION_SHAPE_BROADCASTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

class FormationShapeBroadcaster : public rclcpp::Node
{
public:
    FormationShapeBroadcaster();

private:
    void load_yaml(const std::string &file_path);
    void publish_transforms();

    std::vector<geometry_msgs::msg::TransformStamped> transforms_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // FORMATION_SHAPE_BROADCASTER_HPP