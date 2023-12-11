#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <ctime> 

class Traj2PositionNode : public rclcpp::Node {
public:
    explicit Traj2PositionNode(const rclcpp::NodeOptions& options)
        : Node("traj2position_node", options) {
        from_filename_ = "src/PlannerGAN/data/GAN_trajectory_data.csv";
        to_filename_ = "src/PlannerGAN/data/GAN_end_effector_positions_data.csv";
        GANtraj2position();
    }

    void GANtraj2position() {

        // Allow time for initialization
        rclcpp::sleep_for(std::chrono::seconds(5)); 

        // Check if the file exists
        if (!std::filesystem::exists(from_filename_)) {
            RCLCPP_INFO(this->get_logger(), "File %s does not exist", from_filename_.c_str());
            return; // Exit the function if the file does not exist
        }

        robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
        moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);

        std::ifstream csv_file(from_filename_);
        std::string line;
        std::vector<double> joint_values(7, 0.0); // Assuming 7 joint values per line

        while (std::getline(csv_file, line)) {
            std::stringstream lineStream(line);
            std::string cell;
            int joint_idx = 0;
            while (std::getline(lineStream, cell, ',')) {
                joint_values[joint_idx++] = std::stod(cell);
            }

            kinematic_state->setJointGroupPositions("iiwa_arm", joint_values);
            kinematic_state->enforceBounds();

            const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
            geometry_msgs::msg::Pose pose;
            tf2::convert(end_effector_state, pose);
            end_effector_poses_.push_back(pose);
        }
        csv_file.close();

        save_GAN_end_effector_position_data_to_csv();
    }

private:
    std::string from_filename_;
    std::string to_filename_;
    std::vector<geometry_msgs::msg::Pose> end_effector_poses_;

    void save_GAN_end_effector_position_data_to_csv() {
        std::ofstream csv_file(to_filename_, std::ios::app);

        if (csv_file.is_open()) {
            for (const auto& pose : end_effector_poses_) {
                csv_file << pose.position.x << "," << pose.position.y << "," << pose.position.z << "\n";
            }
            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "End effector position data saved to %s", to_filename_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", to_filename_.c_str());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Traj2PositionNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
