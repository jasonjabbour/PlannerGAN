#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>
#include <fstream>
#include <filesystem>

// Define the class for the motion planning node
class MotionPlanningNode : public rclcpp::Node {
public:
    // Constructor for the node
    explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
    : Node("motion_planning_node", options) {}

    // Main planning function
    void start_planning() {
        // Make pair_id static so its value is retained across function calls
        static int pair_id = 0;  

        // Separate vectors for RRT and CHOMP end effector poses
        std::vector<std::vector<geometry_msgs::msg::Pose>> rrt_joint_poses;
        std::vector<std::vector<geometry_msgs::msg::Pose>> chomp_joint_poses;

        // 2D Vector to store poses for each joint at each point in the trajectory
        std::vector<std::vector<geometry_msgs::msg::Pose>> all_joint_poses;

        // Declare plans for RRT and CHOMP
        moveit::planning_interface::MoveGroupInterface::Plan rrt_plan;
        moveit::planning_interface::MoveGroupInterface::Plan chomp_plan;

        // Initialize the MoveGroupInterface for the robot arm
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");
        rclcpp::sleep_for(std::chrono::seconds(5)); // Allow time for initialization

        // Define the target pose for the robot's end effector
        geometry_msgs::msg::Pose target_pose = create_target_pose();

        // Variables to store various metrics
        double path_length, smoothness;
        long int planning_time, execution_time;
        bool rrt_success = false, chomp_success = false; // Flags for the success of each planning algorithm

        // Execute planning using the RRT algorithm
        RCLCPP_INFO(this->get_logger(), "Starting planning with RRT...");
        rrt_success = plan_and_execute(move_group, "RRTkConfigDefault", target_pose, all_joint_poses, path_length, smoothness, planning_time, execution_time, rrt_plan);
        RCLCPP_INFO(this->get_logger(), "RRT Planning and execution complete");

        // Execute planning using RRT and save the poses if successful
        if (rrt_success) {
            rrt_joint_poses = all_joint_poses; // Save RRT poses
        }

        // Reset the robot to its start position
        reset_to_start_position(move_group);

        // Execute planning using the CHOMP algorithm
        RCLCPP_INFO(this->get_logger(), "Starting planning with CHOMP...");
        chomp_success = plan_and_execute(move_group, "CHOMPkConfigDefault", target_pose, all_joint_poses, path_length, smoothness, planning_time, execution_time, chomp_plan);
        RCLCPP_INFO(this->get_logger(), "CHOMP Planning and execution complete");

        // Execute planning using CHOMP and save the poses if successful
        if (chomp_success) {
            chomp_joint_poses = all_joint_poses; // Save CHOMP poses
        }

        // Save data to CSV if both plans are successful
        if (rrt_success && chomp_success) {
            // Save trajectory data to CSV
            save_trajectory_data_to_csv(rrt_plan, chomp_plan, pair_id); 
            // Save joint position data to CSV
            save_joint_position_data_to_csv(rrt_joint_poses, chomp_joint_poses, pair_id);
            // Increment pair id
            pair_id++; 
        }

    }

private:

    // Function to execute a planning algorithm and move the robot
    bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group, 
                          const std::string& planner_id, 
                          const geometry_msgs::msg::Pose& target_pose, 
                          std::vector<std::vector<geometry_msgs::msg::Pose>>& all_joint_poses,
                          double& path_length, 
                          double& smoothness, 
                          long int& planning_time, 
                          long int& execution_time, 
                          moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        // Set the planner ID and target pose for planning
        move_group.setPlannerId(planner_id);
        move_group.setPoseTarget(target_pose);

        // Perform the planning
        auto start_time = std::chrono::steady_clock::now(); // Start timing the planning
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        auto end_time = std::chrono::steady_clock::now(); // End timing the planning
        planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count(); // Calculate planning time

        // If planning was successful, execute the plan
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
            start_time = std::chrono::steady_clock::now(); // Start timing the execution
            move_group.move(); // Execute the plan
            end_time = std::chrono::steady_clock::now(); // End timing the execution
            execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count(); // Calculate execution time

            // Get all joint positions from the plan
            traj2allJointPositions(plan, all_joint_poses);
 
            // Analyze the trajectory for path length and smoothness
            analyze_trajectory(plan.trajectory_.joint_trajectory, path_length, smoothness);

            RCLCPP_INFO(this->get_logger(), "Path Length: %f, Smoothness: %f", path_length, smoothness);
            RCLCPP_INFO(this->get_logger(), "Planning Time: %ld ms, Execution Time: %ld ms", planning_time, execution_time);
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning failed.");
        }

        return success;
    }


    // Function to convert joint trajectory to all joint positions
    void traj2allJointPositions(const moveit::planning_interface::MoveGroupInterface::Plan& plan, 
                                std::vector<std::vector<geometry_msgs::msg::Pose>>& all_joint_poses) {
        // Load the robot model to perform kinematic calculations
        robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
        moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);

        // Correct link names as per your URDF
        std::vector<std::string> link_names = {"link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_7"};

        // Resize the vector to hold poses for each link
        all_joint_poses.clear();
        all_joint_poses.resize(plan.trajectory_.joint_trajectory.points.size(), std::vector<geometry_msgs::msg::Pose>(link_names.size()));

        // Iterate over each point in the trajectory
        for (size_t point_idx = 0; point_idx < plan.trajectory_.joint_trajectory.points.size(); ++point_idx) {
            const auto& point = plan.trajectory_.joint_trajectory.points[point_idx];
            
            // Set the robot's joint positions
            kinematic_state->setJointGroupPositions("iiwa_arm", point.positions);
            kinematic_state->enforceBounds();

            // Get the pose for each link
            for (size_t link_idx = 0; link_idx < link_names.size(); ++link_idx) {
                try {
                    const Eigen::Isometry3d& link_state = kinematic_state->getGlobalLinkTransform(link_names[link_idx]);
                    geometry_msgs::msg::Pose pose;
                    tf2::convert(link_state, pose);
                    all_joint_poses[point_idx][link_idx] = pose;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error getting transform for link '%s': %s", link_names[link_idx].c_str(), e.what());
                }
            }
        }
    }

    // Function to create a target pose for the end effector
    geometry_msgs::msg::Pose create_target_pose() {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.4;
        pose.position.y = 0.1;
        pose.position.z = 0.7;
        pose.orientation.w = 1.0;
        return pose;
    }

    // Function to reset the robot to its start position
    void reset_to_start_position(moveit::planning_interface::MoveGroupInterface& move_group) {
        // Define joint positions for the start position
        std::map<std::string, double> joint_values;
        joint_values["joint_a1"] = 0.0;
        joint_values["joint_a2"] = -0.7854;
        joint_values["joint_a3"] = 0.0;
        joint_values["joint_a4"] = 1.3962;
        joint_values["joint_a5"] = 0.0;
        joint_values["joint_a6"] = 0.6109;
        joint_values["joint_a7"] = 0.0;

        // Set the target joint values and move the robot
        move_group.setJointValueTarget(joint_values);
        move_group.move();
    }

    // Function to analyze the trajectory for path length and smoothness
    void analyze_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                            double& path_length, double& smoothness) {
        path_length = 0.0; // Reset path length
        smoothness = 0.0;  // Reset smoothness

        // Calculate the path length
        for (size_t i = 1; i < trajectory.points.size(); ++i) {
            const auto& prev_point = trajectory.points[i - 1];
            const auto& current_point = trajectory.points[i];

            double segment_length = 0.0; // Initialize segment length
            // Calculate the Euclidean distance between consecutive points
            for (size_t j = 0; j < prev_point.positions.size(); ++j) {
                segment_length += std::pow(current_point.positions[j] - prev_point.positions[j], 2);
            }
            path_length += std::sqrt(segment_length);
        }
        // Calculate smoothness (implementation depends on your definition of smoothness)
    }


    // Function to save the trajectory data to a CSV 
    void save_trajectory_data_to_csv(const moveit::planning_interface::MoveGroupInterface::Plan& rrt_plan, 
                                 const moveit::planning_interface::MoveGroupInterface::Plan& chomp_plan, 
                                 int pair_id) {
        const std::string filename = "src/PlannerGAN/data/trajectory_data.csv";
        std::ofstream csv_file(filename, std::ios::app); // Open file in append mode

        bool write_header = !std::filesystem::exists(filename) || std::filesystem::file_size(filename) == 0;

        if (csv_file.is_open()) {
            if (write_header) {
                csv_file << "PairID,Algorithm";
                for (int i = 1; i <= 7; ++i) {
                    csv_file << ",Joint" << i;
                }
                csv_file << "\n";
            }

            // Save RRT trajectory
            for (const auto& point : rrt_plan.trajectory_.joint_trajectory.points) {
                csv_file << pair_id << ",RRT";
                for (const auto& position : point.positions) {
                    csv_file << "," << position;
                }
                csv_file << "\n";
            }

            // Save CHOMP trajectory
            for (const auto& point : chomp_plan.trajectory_.joint_trajectory.points) {
                csv_file << pair_id << ",CHOMP";
                for (const auto& position : point.positions) {
                    csv_file << "," << position;
                }
                csv_file << "\n";
            }

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory data saved to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", filename.c_str());
        }
    }


    // Function to save the joint position data to CSV
    void save_joint_position_data_to_csv(const std::vector<std::vector<geometry_msgs::msg::Pose>>& rrt_poses, 
                        const std::vector<std::vector<geometry_msgs::msg::Pose>>& chomp_poses, 
                        int pair_id) {
        const std::string filename = "src/PlannerGAN/data/joint_positions_data.csv";
        std::ofstream csv_file(filename, std::ios::app); // Open file in append mode

        bool write_header = !std::filesystem::exists(filename) || std::filesystem::file_size(filename) == 0;

        if (csv_file.is_open()) {
            if (write_header) {
                csv_file << "PairID,Algorithm";
                for (int i = 1; i <= 7; ++i) {
                    csv_file << ",Joint" << i << "_X,Joint" << i << "_Y,Joint" << i << "_Z";
                }
                csv_file << "\n";
            }

            for (size_t i = 0; i < rrt_poses.size(); ++i) { // Iterate over each point in the trajectory
                csv_file << pair_id << ",RRT";
                for (size_t j = 0; j < rrt_poses[i].size(); ++j) { // Iterate over each joint
                    const auto& pose = rrt_poses[i][j];
                    csv_file << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z;
                }
                csv_file << "\n";
            }

            for (size_t i = 0; i < chomp_poses.size(); ++i) { // Similarly for CHOMP
                csv_file << pair_id << ",CHOMP";
                for (size_t j = 0; j < chomp_poses[i].size(); ++j) {
                    const auto& pose = chomp_poses[i][j];
                    csv_file << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z;
                }
                csv_file << "\n";
            }

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory data saved to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", filename.c_str());
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

