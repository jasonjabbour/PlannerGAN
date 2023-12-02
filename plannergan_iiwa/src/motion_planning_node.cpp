#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <chrono>

class MotionPlanningNode : public rclcpp::Node {
public:
    explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
    : Node("motion_planning_node", options) {}

    void start_planning() {
        // Initialize MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

        // Sleep to allow everything to initialize
        rclcpp::sleep_for(std::chrono::seconds(5));

        // Define the target pose for the end effector
        geometry_msgs::msg::Pose target_pose = create_target_pose();

        double path_length, smoothness;
        long int planning_time, execution_time;

        // Plan and execute using RRT
        RCLCPP_INFO(this->get_logger(), "Starting planning with RRT...");
        plan_and_execute(move_group, "RRTkConfigDefault", target_pose, path_length, smoothness, planning_time, execution_time);
        RCLCPP_INFO(this->get_logger(), "RRT Planning and execution complete");

        // Reset arm to start position
        RCLCPP_INFO(this->get_logger(), "Resetting to start position...");
        reset_to_start_position(move_group);

        // Plan and execute using CHOMP
        RCLCPP_INFO(this->get_logger(), "Starting planning with CHOMP...");
        plan_and_execute(move_group, "CHOMPkConfigDefault", target_pose, path_length, smoothness, planning_time, execution_time); // Assuming this is your CHOMP config name
        RCLCPP_INFO(this->get_logger(), "CHOMP Planning and execution complete");
    }

private:
    void plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group, 
                          const std::string& planner_id, 
                          const geometry_msgs::msg::Pose& target_pose, 
                          double& path_length, 
                          double& smoothness, 
                          long int& planning_time, 
                          long int& execution_time) {
        // Set the planner ID and target pose
        move_group.setPlannerId(planner_id);
        move_group.setPoseTarget(target_pose);

        // Perform planning
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto start_time = std::chrono::steady_clock::now();
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        auto end_time = std::chrono::steady_clock::now();
        planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
            start_time = std::chrono::steady_clock::now();
            move_group.move(); // Execute the plan
            end_time = std::chrono::steady_clock::now();
            execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            // Analyze the trajectory
            analyze_trajectory(plan.trajectory_.joint_trajectory, path_length, smoothness);
            RCLCPP_INFO(this->get_logger(), "Path Length: %f, Smoothness: %f", path_length, smoothness);
            RCLCPP_INFO(this->get_logger(), "Planning Time: %ld ms, Execution Time: %ld ms", planning_time, execution_time);
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

    void analyze_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                            double& path_length, 
                            double& smoothness) {
        path_length = 0.0;
        smoothness = 0.0;

        // Example calculation for path length
        for (size_t i = 1; i < trajectory.points.size(); ++i) {
            const auto& prev_point = trajectory.points[i - 1];
            const auto& current_point = trajectory.points[i];

            double segment_length = 0.0;
            for (size_t j = 0; j < prev_point.positions.size(); ++j) {
                segment_length += std::pow(current_point.positions[j] - prev_point.positions[j], 2);
            }
            path_length += std::sqrt(segment_length);
        }

        // Example calculation for smoothness (simple version)
        // ... [implement your smoothness calculation here]

        RCLCPP_INFO(this->get_logger(), "Path Length: %f, Smoothness: %f", path_length, smoothness);
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
//     : Node("motion_planning_node", options) {}

//     void start_planning() {
//         // Create the MoveGroupInterface after the node has been constructed
//         moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

//         // Sleep to allow everything to initialize
//         rclcpp::sleep_for(std::chrono::seconds(5));

//         // Define the target pose for the end effector
//         geometry_msgs::msg::Pose target_pose = create_target_pose();

//         // Plan and execute using RRT
//         plan_and_execute(move_group, "RRTkConfigDefault", target_pose);

//         RCLCPP_INFO(this->get_logger(), "!!!!RRT Arrived at Target!!!!");
//         rclcpp::sleep_for(std::chrono::seconds(10));

//         // Reset arm to start position 
//         reset_to_start_position(move_group);

//         RCLCPP_INFO(this->get_logger(), "!!!!Reset to Original Position!!!!");
//         rclcpp::sleep_for(std::chrono::seconds(10));

//         // Plan and execute using CHOMP
//         plan_and_execute(move_group, "CHOMPkConfigDefault", target_pose); // Assuming this is your CHOMP config name
    
//         RCLCPP_INFO(this->get_logger(), "!!!!CHOMP Arrived at Target!!!!");

//     }

// private:
//     void plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& planner_id, const geometry_msgs::msg::Pose& target_pose) {
//         RCLCPP_INFO(this->get_logger(), "Planning with %s", planner_id.c_str());
//         move_group.setPlannerId(planner_id);
//         move_group.setPoseTarget(target_pose);

//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (success) {
//             RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
//             move_group.move(); // Execute the plan
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Planning failed.");
//         }
//     }

//     geometry_msgs::msg::Pose create_target_pose() {
//         geometry_msgs::msg::Pose pose;
//         pose.position.x = 0.4;
//         pose.position.y = 0.1;
//         pose.position.z = 0.7;
//         pose.orientation.w = 1.0;
//         return pose;
//     }

//     void reset_to_start_position(moveit::planning_interface::MoveGroupInterface& move_group) {
//         RCLCPP_INFO(this->get_logger(), "Resetting to start position using joint values");

//         std::map<std::string, double> joint_values;
//         joint_values["joint_a1"] = 0.0;
//         joint_values["joint_a2"] = -0.7854;
//         joint_values["joint_a3"] = 0.0;
//         joint_values["joint_a4"] = 1.3962;
//         joint_values["joint_a5"] = 0.0;
//         joint_values["joint_a6"] = 0.6109;
//         joint_values["joint_a7"] = 0.0;

//         move_group.setJointValueTarget(joint_values);
//         bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

//         if (success) {
//             RCLCPP_INFO(this->get_logger(), "Robot successfully reset to start position");
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Failed to reset robot to start position");
//         }
//     }

// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MotionPlanningNode>(rclcpp::NodeOptions());
//     node->start_planning();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


