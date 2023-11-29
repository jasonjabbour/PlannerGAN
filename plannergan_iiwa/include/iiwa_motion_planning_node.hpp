// iiwa_motion_planning_node.hpp
#ifndef IIWA_MOTION_PLANNING_NODE_HPP
#define IIWA_MOTION_PLANNING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class IIWAMotionPlanningNode : public rclcpp::Node {
public:
    IIWAMotionPlanningNode();
    void initialize();

private:
    void planAndMove(const geometry_msgs::msg::Pose& target_pose);
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};

#endif // IIWA_MOTION_PLANNING_NODE_HPP
