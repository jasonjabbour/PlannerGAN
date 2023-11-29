// iiwa_motion_planning_node.cpp
#include "iiwa_motion_planning_node.hpp"

IIWAMotionPlanningNode::IIWAMotionPlanningNode() : Node("iiwa_motion_planning_node") {
    // Constructor logic here
}

void IIWAMotionPlanningNode::initialize() {
    // Initialize Move Group
    this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "iiwa_arm");

    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4; // example values
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.5;
    target_pose.orientation.w = 1.0;

    // Perform motion planning
    planAndMove(target_pose);
}

void IIWAMotionPlanningNode::planAndMove(const geometry_msgs::msg::Pose& target_pose) {
    move_group->setPlannerId("RRTkConfigDefault");

    move_group->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
        move_group->execute(my_plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IIWAMotionPlanningNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
