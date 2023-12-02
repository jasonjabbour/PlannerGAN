#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

class MotionPlanningNode : public rclcpp::Node {
public:
    explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
    : Node("motion_planning_node", options) {}

    void start_planning() {
        // Create the MoveGroupInterface after the node has been constructed
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

        // Sleep to allow everything to initialize
        rclcpp::sleep_for(std::chrono::seconds(5));

        // Define the target pose for the end effector
        geometry_msgs::msg::Pose target_pose = create_target_pose();

        // Plan and execute using RRT
        plan_and_execute(move_group, "RRTkConfigDefault", target_pose);

        RCLCPP_INFO(this->get_logger(), "!!!!RRT Arrived at Target!!!!");
        rclcpp::sleep_for(std::chrono::seconds(10));

        // Reset arm to start position 
        reset_to_start_position(move_group);

        RCLCPP_INFO(this->get_logger(), "!!!!Reset to Original Position!!!!");
        rclcpp::sleep_for(std::chrono::seconds(10));

        // Plan and execute using CHOMP
        plan_and_execute(move_group, "CHOMPkConfigDefault", target_pose); // Assuming this is your CHOMP config name
    
        RCLCPP_INFO(this->get_logger(), "!!!!CHOMP Arrived at Target!!!!");

    }

private:
    void plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& planner_id, const geometry_msgs::msg::Pose& target_pose) {
        RCLCPP_INFO(this->get_logger(), "Planning with %s", planner_id.c_str());
        move_group.setPlannerId(planner_id);
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
            move_group.move(); // Execute the plan
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning failed.");
        }
    }

    geometry_msgs::msg::Pose create_target_pose() {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.4;
        pose.position.y = 0.1;
        pose.position.z = 0.7;
        pose.orientation.w = 1.0;
        return pose;
    }

    void reset_to_start_position(moveit::planning_interface::MoveGroupInterface& move_group) {
        RCLCPP_INFO(this->get_logger(), "Resetting to start position using joint values");

        std::map<std::string, double> joint_values;
        joint_values["joint_a1"] = 0.0;
        joint_values["joint_a2"] = -0.7854;
        joint_values["joint_a3"] = 0.0;
        joint_values["joint_a4"] = 1.3962;
        joint_values["joint_a5"] = 0.0;
        joint_values["joint_a6"] = 0.6109;
        joint_values["joint_a7"] = 0.0;

        move_group.setJointValueTarget(joint_values);
        bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Robot successfully reset to start position");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to reset robot to start position");
        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningNode>(rclcpp::NodeOptions());
    node->start_planning();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <chrono>

// class MotionPlanningNode : public rclcpp::Node {
// public:
//     explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
//     : Node("motion_planning_node", options) {
//         // Defer move_group initialization to start_planning()
//     }

//     void start_planning() {
//         // Create the MoveGroupInterface after the node has been constructed
//         moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

//         // Sleep for 10 seconds to allow everything to initialize
//         rclcpp::sleep_for(std::chrono::seconds(10));

//         // Define a fixed, known-good target pose for the end effector
//         geometry_msgs::msg::Pose target_pose;
//         target_pose.position.x = 0.4;
//         target_pose.position.y = 0.1;
//         target_pose.position.z = 0.7;
//         target_pose.orientation.w = 1.0;

//         // Set the target pose
//         move_group.setPoseTarget(target_pose);

//         // Set the planner to RRT
//         move_group.setPlannerId("RRTkConfigDefault");

//         // Perform planning
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//         bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (success) {
//             RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
//             move_group.move(); // Execute the plan
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Planning failed.");
//         }
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MotionPlanningNode>(rclcpp::NodeOptions());

//     // Start the planning process immediately
//     node->start_planning();

//     rclcpp::spin(node); // Handle ROS callbacks
//     rclcpp::shutdown();
//     return 0;
// }
