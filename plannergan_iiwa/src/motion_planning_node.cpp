#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread> // Include this header for std::this_thread::sleep_for

class MotionPlanningNode : public rclcpp::Node
{
public:
    MotionPlanningNode() : Node("motion_planning_node")
    {
        // Use a timer to delay the initialization of MoveIt components
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&MotionPlanningNode::init, this));
    }

private:
    void init()
    {
        // Initialize MoveIt
        auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "iiwa_arm");

        // Define the target pose or joint values
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 1.0;  // Example values
        target_pose.position.y = 0.2;
        target_pose.position.z = 0.5;
        target_pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "********************Waiting before planning...");

        // Delay for a specified amount of time (e.g., 5 seconds)
        std::this_thread::sleep_for(std::chrono::seconds(10));

        RCLCPP_INFO(this->get_logger(), "Resuming planning...");

        // Set the target
        move_group_interface->setPoseTarget(target_pose);
        // Set the Planning Algorithm 
        move_group_interface->setPlannerId("RRTConnectkConfigDefault");

        // Plan to the new pose
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            move_group_interface->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }

        // Shutdown the node after execution
        rclcpp::shutdown();
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}