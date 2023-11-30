#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

class MotionPlanningNode : public rclcpp::Node
{
public:
    MotionPlanningNode() : Node("motion_planning_node") {}

    static std::shared_ptr<MotionPlanningNode> create()
    {
        auto node = std::make_shared<MotionPlanningNode>();
        node->initialize();
        return node;
    }

private:
    void initialize()
    {
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");

        while (rclcpp::ok()) {
            // Sample a random valid pose
            geometry_msgs::msg::Pose target_pose = getRandomValidPose(move_group);

            // Set the target pose
            move_group.setPoseTarget(target_pose);

            // Perform planning
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
                move_group.execute(my_plan);
                break;  // Exit the loop if the plan is successful
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning failed, trying a new pose...");
            }

            // Add a short delay to allow for user interruption or system responsiveness
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    geometry_msgs::msg::Pose getRandomValidPose(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());

        // Get a random joint configuration
        auto current_state = move_group.getCurrentState();
        current_state->setToRandomPositions(joint_model_group);

        // Get the pose of the end effector for this random joint configuration
        const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform(move_group.getEndEffectorLink());

        // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose pose;
        tf2::convert(end_effector_state, pose);

        return pose;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = MotionPlanningNode::create();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <thread> // Include for using sleep functionalities

// // Define the MotionPlanningNode class which inherits from rclcpp::Node
// class MotionPlanningNode : public rclcpp::Node
// {
// public:
//     // Constructor for the node
//     MotionPlanningNode() : Node("motion_planning_node")
//     {
//         // Set up a timer to delay initialization of MoveIt components
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100), 
//             std::bind(&MotionPlanningNode::init, this));
//     }

// private:
//     // Initialization function for MoveIt components
//     void init()
//     {
//         // Initialize the Move Group Interface for controlling the robot
//         auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             shared_from_this(), "iiwa_arm");

//         // // Log information indicating planning is about to start
//         // RCLCPP_INFO(this->get_logger(), "********************Waiting before planning...");
        
//         // // Delay for a specified amount of time to allow for setup or observation
//         // std::this_thread::sleep_for(std::chrono::seconds(10));
        
//         // // Log information indicating planning is resuming
//         // RCLCPP_INFO(this->get_logger(), "Resuming planning...");

//         // Set the planner ID to be used for motion planning
//         move_group_interface->setPlannerId("RRTConnectkConfigDefault");

//         // auto current_state = move_group_interface->getCurrentState(10);
//         // if (!current_state)
//         // {
//         //     RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
//         //     rclcpp::shutdown();
//         //     return;
//         // }

//         // Define a variable to hold the plan
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//         bool success = false;

//         // Keep trying until a successful plan is found
//         while (!success)
//         {
//             // Generate a random target pose that is valid for the robot
//             auto target_pose = move_group_interface->getRandomPose();

//             // Set the randomly generated pose as the new target
//             move_group_interface->setPoseTarget(target_pose);

//             // Attempt to plan to the new pose
//             success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//             // Check if the planning was successful
//             if (success)
//             {
//                 // Log success and execute the plan
//                 RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
//                 move_group_interface->execute(my_plan);
//             }
//             else
//             {
//                 // Log failure and try another random pose
//                 RCLCPP_INFO(this->get_logger(), "Planning failed, trying another random pose...");
//             }
//         }

//         // Shutdown the node after execution
//         rclcpp::shutdown();
//     }

//     // Timer to delay the initialization
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// // Main function
// int main(int argc, char **argv)
// {
//     // Initialize the ROS 2 node
//     rclcpp::init(argc, argv);

//     // Create an instance of the MotionPlanningNode
//     auto node = std::make_shared<MotionPlanningNode>();

//     // Spin the node to process callbacks
//     rclcpp::spin(node);

//     // Shutdown ROS 2 upon exiting the node
//     rclcpp::shutdown();

//     return 0;
// }
