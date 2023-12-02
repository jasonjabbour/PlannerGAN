#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

class MotionPlanningNode : public rclcpp::Node {
public:
    explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
    : Node("motion_planning_node", options) {
        // Defer move_group initialization to start_planning()
    }

    void start_planning() {
        // Create the MoveGroupInterface after the node has been constructed
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

        // Sleep for 10 seconds to allow everything to initialize
        rclcpp::sleep_for(std::chrono::seconds(10));

        // Define a fixed, known-good target pose for the end effector
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.7;
        target_pose.orientation.w = 1.0;

        // Set the target pose
        move_group.setPoseTarget(target_pose);

        // Set the planner to RRT
        move_group.setPlannerId("RRTkConfigDefault");

        // Perform planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            move_group.move(); // Execute the plan
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning failed.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningNode>(rclcpp::NodeOptions());

    // Start the planning process immediately
    node->start_planning();

    rclcpp::spin(node); // Handle ROS callbacks
    rclcpp::shutdown();
    return 0;
}
