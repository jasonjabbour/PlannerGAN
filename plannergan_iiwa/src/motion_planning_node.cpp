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

// Define the class for the motion planning node
class MotionPlanningNode : public rclcpp::Node {
public:

    // Public member variable
    int pair_id = 0;

    bool first = true;
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose target_pose;
    // Define the names and ids of the two algorithms 
    std::string algo1_name = "RRT";
    std::string algo1_planner_id = "RRTkConfigDefault";

    std::string algo2_name = "RRT*";
    std::string algo2_planner_id = "RRTstarkConfigDefault";

    // std::string algo2_name = "CHOMP";
    // std::string algo2_planner_id = "CHOMPkConfigDefault";

    // Constructor for the node
    explicit MotionPlanningNode(const rclcpp::NodeOptions& options)
    : Node("motion_planning_node", options) {
        
        // File name to get the last pair id created
        const std::string filename = "src/PlannerGAN/data/trajectory_data.csv";
        // const std::string filename = "trajectory_data.csv";
        // Update the pair id
        pair_id = get_last_pair_id_from_csv(filename) + 1;

    }

    // Run the planning function n times
    void start_planning_n_samples(int n_iterations, int n_samples) {
        // Initialize the MoveGroupInterface for the robot arm
        moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), "iiwa_arm");
        rclcpp::sleep_for(std::chrono::seconds(5)); // Allow time for initialization
        
        for (int i = 0; i < n_iterations; ++i) {
            // FLOW: Move arm to initial start position if first run
            if (first) {
                start_pose = create_random_pose(move_group);
                first = false;
            } else {
                start_pose = target_pose;
            }
            // FLOW: Set target pose to new random pose
            target_pose = create_random_pose(move_group);
            // FLOW: Run n sample iterations with these start and end pose parameters
            for (int j = 0; j < n_samples; ++j) {
                RCLCPP_INFO(this->get_logger(), "\n\n\n\n****Iteration Number: %d. Sample Number: %d. Pair ID: %d****", i, j, pair_id);
                start_planning(move_group);
            }
        }
    }

    // Main planning function
    void start_planning(moveit::planning_interface::MoveGroupInterface& move_group) {
        // Separate vectors for the two algorithm's end effector poses
        std::vector<std::vector<geometry_msgs::msg::Pose>> algo1_joint_poses;
        std::vector<std::vector<geometry_msgs::msg::Pose>> algo2_joint_poses;

        // Vector to store poses of the end effector for each point in the trajectory
        std::vector<geometry_msgs::msg::Pose> algo1_end_effector_poses;
        std::vector<geometry_msgs::msg::Pose> algo2_end_effector_poses;

        // 2D Vector to store poses for each joint at each point in the trajectory
        std::vector<std::vector<geometry_msgs::msg::Pose>> all_joint_poses;
        // Vector to store end effector poses at each point in the trajectory
        std::vector<geometry_msgs::msg::Pose> end_effector_poses;

        // Declare plans for both algorithms
        moveit::planning_interface::MoveGroupInterface::Plan algo1_plan;
        moveit::planning_interface::MoveGroupInterface::Plan algo2_plan;

        // Declare initial plan reset plan
        moveit::planning_interface::MoveGroupInterface::Plan reset_plan;

        // Add obstacles to the planning scene
        add_obstacles(move_group);

        // Variables to store various metrics
        double algo1_path_length, algo1_smoothness, algo2_path_length, algo2_smoothness;
        long int algo1_planning_time, algo1_execution_time, algo2_planning_time, algo2_execution_time;
        bool algo1_success = false, algo2_success = false; // Flags for the success of each planning algorithm
        bool first_reset_success = false, second_reset_success = false;

        // Reset the robot to its start position
        first_reset_success = reset_to_start_position(move_group, reset_plan);

        // Execute planning using the RRT algorithm
        if (first_reset_success) {
            RCLCPP_INFO(this->get_logger(), "Starting planning with %s...", algo1_name.c_str());
            algo1_success = plan_and_execute(move_group,
                                        algo1_planner_id,
                                        target_pose,
                                        end_effector_poses,
                                        all_joint_poses,
                                        algo1_path_length,
                                        algo1_smoothness,
                                        algo1_planning_time,
                                        algo1_execution_time,
                                        algo1_plan);
            RCLCPP_INFO(this->get_logger(), "%s Planning and execution complete", algo1_name.c_str());
        }

        // Save the poses if successful
        if (algo1_success) {
            // Save poses
            algo1_joint_poses = all_joint_poses; 
            // Save end effector poses
            algo1_end_effector_poses = end_effector_poses; 
        }

        // Reset the robot to its start position
        second_reset_success = reset_to_start_position(move_group, reset_plan);

        if (second_reset_success) {
            // Execute planning using the algo2
            RCLCPP_INFO(this->get_logger(), "Starting planning with %s...", algo2_name.c_str());
            algo2_success = plan_and_execute(move_group,
                                            algo2_planner_id, 
                                            target_pose, 
                                            end_effector_poses,
                                            all_joint_poses, 
                                            algo2_path_length, 
                                            algo2_smoothness, 
                                            algo2_planning_time, 
                                            algo2_execution_time, 
                                            algo2_plan);
            RCLCPP_INFO(this->get_logger(), "%s Planning and execution complete", algo2_name.c_str());
        }

        // Save the poses if successful
        if (algo2_success) {
            // Save poses
            algo2_joint_poses = all_joint_poses;
            // Save end effector poses
            algo2_end_effector_poses = end_effector_poses; 
        }

        // Save data to CSV if both plans are successful
        if (algo1_success && algo2_success/*&& (chomp_path_length < rrt_path_length) && (chomp_planning_time < rrt_planning_time) && (chomp_execution_time < rrt_execution_time)*/) {
            RCLCPP_INFO(this->get_logger(), "\nAll checks passed, saving data to csv...");
            // Save trajectory data to CSV
            save_trajectory_data_to_csv(algo1_plan, algo2_plan, pair_id); 
            // Save end effector data to CSV
            save_end_effector_position_data_to_csv(algo1_end_effector_poses, algo2_end_effector_poses, pair_id);
            // Save joint position data to CSV
            save_joint_position_data_to_csv(algo1_joint_poses, algo2_joint_poses, pair_id);
            // Save metrics to CSV
            save_metrics_to_csv(algo1_name, pair_id, algo1_planning_time, algo1_execution_time, algo1_smoothness, algo1_path_length);
            save_metrics_to_csv(algo2_name, pair_id, algo2_planning_time, algo2_execution_time, algo2_smoothness, algo2_path_length);
            // Increment pair id
            pair_id++; 
        } else {
            RCLCPP_WARN(this->get_logger(), "\n***FILTERED OUT PAIR ID: %d***", pair_id);
        }

    }

    void add_obstacles(moveit::planning_interface::MoveGroupInterface& move_group) {

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        // Function to create a box obstacle
        auto create_box_obstacle = [&](const std::string& id, double x, double y, double z, 
                                    double size_x, double size_y, double size_z) {
            moveit_msgs::msg::CollisionObject box;
            box.header.frame_id = move_group.getPlanningFrame();
            box.id = id;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {size_x, size_y, size_z};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = x;
            box_pose.position.y = y;
            box_pose.position.z = z;

            box.primitives.push_back(primitive);
            box.primitive_poses.push_back(box_pose);
            box.operation = box.ADD;

            return box;
        };

        // Add several boxes to the environment
        // collision_objects.push_back(create_box_obstacle("box1", 0.0, -0.4, 0.5, 0.4, 0.4, 0.4));
        // collision_objects.push_back(create_box_obstacle("box2", 0.5, -0.4, 0.2, 0.3, 0.3, 0.3));
        // collision_objects.push_back(create_box_obstacle("box3", -0.5, 0.4, 0.3, 0.2, 0.2, 0.2));

        // Add a floor to the environment
        collision_objects.push_back(create_box_obstacle("floor", 1.0, 0.0, 0.05, 3.0, 3.0, 0.1));


        planning_scene_interface.applyCollisionObjects(collision_objects);

    }


private:

    // Function to execute a planning algorithm and move the robot
    bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group, 
                          const std::string& planner_id, 
                          const geometry_msgs::msg::Pose& target_pose, 
                          std::vector<geometry_msgs::msg::Pose>& end_effector_poses,
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
            RCLCPP_INFO(this->get_logger(), "\nPlan successful, executing...");
            start_time = std::chrono::steady_clock::now(); // Start timing the execution
            move_group.move(); // Execute the plan
            end_time = std::chrono::steady_clock::now(); // End timing the execution
            execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count(); // Calculate execution time

            // Get end effectors positions from the plan
            traj2position(plan, end_effector_poses);

            // Get all joint positions from the plan
            traj2allJointPositions(plan, all_joint_poses);
 
            // Analyze the trajectory for path length and smoothness
            analyze_trajectory(plan.trajectory_.joint_trajectory, path_length, smoothness);

            RCLCPP_INFO(this->get_logger(), "\nPath Length: %f, Smoothness: %f", path_length, smoothness);
            RCLCPP_INFO(this->get_logger(), "\nPlanning Time: %ld ms, Execution Time: %ld ms", planning_time, execution_time);
        } else {
            RCLCPP_WARN(this->get_logger(), "\nPlanning failed.");
        }

        return success;
    }

    // Function to convert joint trajectory the end effector position
    void traj2position(const moveit::planning_interface::MoveGroupInterface::Plan& plan, 
                       std::vector<geometry_msgs::msg::Pose>& end_effector_poses){
        // Load the robot model
        robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
        moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        // Create object to perform kinematic calculations
        auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);

        // Clear the vector
        end_effector_poses.clear();

        // For each trajectory point, set the joint values and compute the forward kinematics
        for (const auto& point : plan.trajectory_.joint_trajectory.points) {
            kinematic_state->setJointGroupPositions("iiwa_arm", point.positions);
            kinematic_state->enforceBounds();
            // Replace "tool0" with your end-effector link name
            const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0"); 

            // Convert Eigen::Isometry3d to geometry_ms gs::msg::Pose
            geometry_msgs::msg::Pose pose;
            tf2::convert(end_effector_state, pose);
            end_effector_poses.push_back(pose);
        }
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

    // Function to create a random pose for the end effector
    // Returns valid path using RRT to chosen position
    geometry_msgs::msg::Pose create_random_pose(moveit::planning_interface::MoveGroupInterface& move_group) {
        while (true)
        {
            geometry_msgs::msg::Pose pose;
            double randX = ((double) rand() / (RAND_MAX));
            if (randX > 0.5) { randX += 1; }
            pose.position.x = randX;
            pose.position.y = (((double) rand() / (RAND_MAX)) * 2) - 1;
            pose.position.z = ((double) rand() / (RAND_MAX)) + .1;
            RCLCPP_INFO(this->get_logger(), "Target X: %f", pose.position.x);
            RCLCPP_INFO(this->get_logger(), "Target Y: %f", pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Target Z: %f", pose.position.z);
            pose.orientation.w = 1.0;

            // FLOW: Create plan based on new pose; if valid pose, return plan; if not, try again
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_group.setPlannerId("RRTkConfigDefault");
            move_group.setPoseTarget(pose);

            if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "\nValid position found...");
                return pose;
            } else {
                RCLCPP_WARN(this->get_logger(), "\nPosition invalid, trying again...");
            }
        }
    }

    // Function to reset the robot to its start position
    bool reset_to_start_position(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        // Set the planner ID and target pose for planning
        move_group.setPlannerId("RRTkConfigDefault");
        move_group.setPoseTarget(start_pose);

        // Perform the planning
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // If planning was successful, execute the plan
        if (success) {
            RCLCPP_INFO(this->get_logger(), "\nSetup plan successful, executing...");
            move_group.move(); // Execute the plan
        } else {
            RCLCPP_WARN(this->get_logger(), "\nSetup planning failed.");
        }

        return success;
    }

    // Function to analyze the trajectory for path length and smoothness
    void analyze_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                            double& path_length, double& smoothness) {
        path_length = 0.0; // Reset path length
        smoothness = 0.0;  // Reset smoothness

        if (trajectory.points.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points in trajectory to calculate smoothness.");
            return;
        }

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

        // Calculate smoothness based on acceleration changes
        double total_acceleration_change = 0.0;
        for (size_t i = 2; i < trajectory.points.size(); ++i) {
            const auto& prev_accel = trajectory.points[i - 1].accelerations;
            const auto& curr_accel = trajectory.points[i].accelerations;

            if (prev_accel.size() != curr_accel.size()) {
                RCLCPP_WARN(this->get_logger(), "Inconsistent acceleration vector size.");
                continue;
            }

            double acceleration_change = 0.0;
            for (size_t j = 0; j < curr_accel.size(); ++j) {
                double diff = curr_accel[j] - prev_accel[j];
                acceleration_change += diff * diff;
            }
            total_acceleration_change += std::sqrt(acceleration_change);
        }

        // Calculate average acceleration change per step
        smoothness = total_acceleration_change / (trajectory.points.size() - 2);
    }

    // BUG FIX THIS!!!!!
    // Function to save the trajectory data to a CSV file
    void save_trajectory_data_to_csv(const moveit::planning_interface::MoveGroupInterface::Plan& algo1_plan, 
                                    const moveit::planning_interface::MoveGroupInterface::Plan& algo2_plan, 
                                    int pair_id) {
        const std::string filename = "src/PlannerGAN/data/trajectory_data.csv";
        // const std::string filename = "trajectory_data.csv";
        std::ofstream csv_file(filename, std::ios::app); // Open file in append mode

        // Check if the file needs a header
        bool write_header = !std::filesystem::exists(filename) || std::filesystem::file_size(filename) == 0;

        if (csv_file.is_open()) {
            // Write the header if needed
            if (write_header) {
                csv_file << "PairID,Algorithm,Time";
                for (int i = 1; i <= 7; ++i) {
                    csv_file << ",Joint" << i << "_Position,Joint" << i << "_Velocity,Joint" << i << "_Acceleration";
                }
                csv_file << "\n";
            }

            // Function to write a single trajectory point to the CSV file
            auto write_trajectory_point = [&](const auto& point, const std::string& algorithm) {
                double time_in_seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
                csv_file << pair_id << "," << algorithm << "," << time_in_seconds;
                for (const auto& position : point.positions) {
                    csv_file << "," << position;
                }
                for (const auto& velocity : point.velocities) {
                    csv_file << "," << velocity;
                }
                for (const auto& acceleration : point.accelerations) {
                    csv_file << "," << acceleration;
                }
                csv_file << "\n";
            };

            // Save algo1 trajectory points
            for (const auto& point : algo1_plan.trajectory_.joint_trajectory.points) {
                write_trajectory_point(point, algo1_name);
            }

            // Save algo2 trajectory points
            for (const auto& point : algo2_plan.trajectory_.joint_trajectory.points) {
                write_trajectory_point(point, algo2_name);
            }

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory data saved to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", filename.c_str());
        }
    }


    // Function to save the end effector position data to CSV
    void save_end_effector_position_data_to_csv(
        const std::vector<geometry_msgs::msg::Pose>& algo1_end_effector_poses, 
        const std::vector<geometry_msgs::msg::Pose>& algo2_end_effector_poses, 
        int pair_id) {
        
        const std::string filename = "src/PlannerGAN/data/end_effector_positions_data.csv";
        // const std::string filename = "end_effector_positions_data.csv";
        std::ofstream csv_file(filename, std::ios::app); // Open file in append mode

        // Check if the file is new or empty to write the header
        bool write_header = !std::filesystem::exists(filename) || std::filesystem::file_size(filename) == 0;

        if (csv_file.is_open()) {
            // Write header if needed
            if (write_header) {
                csv_file << "PairID,Algorithm,X,Y,Z\n";
            }

            // Save algo1 end effector positions
            for (const auto& pose : algo1_end_effector_poses) {
                csv_file << pair_id << "," << algo1_name << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z << "\n";
            }

            // Save algo2 end effector positions
            for (const auto& pose : algo2_end_effector_poses) {
                csv_file << pair_id << "," << algo2_name << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z << "\n";
            }

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "End effector position data saved to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", filename.c_str());
        }
    }

    // Function to save the joint position data to CSV
    void save_joint_position_data_to_csv(const std::vector<std::vector<geometry_msgs::msg::Pose>>& algo1_poses, 
                        const std::vector<std::vector<geometry_msgs::msg::Pose>>& algo2_poses, 
                        int pair_id) {
        const std::string filename = "src/PlannerGAN/data/joint_positions_data.csv";
        // const std::string filename = "joint_positions_data.csv";
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

            for (size_t i = 0; i < algo1_poses.size(); ++i) { // Iterate over each point in the trajectory
                csv_file << pair_id << "," << algo1_name;
                for (size_t j = 0; j < algo1_poses[i].size(); ++j) { // Iterate over each joint
                    const auto& pose = algo1_poses[i][j];
                    csv_file << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z;
                }
                csv_file << "\n";
            }

            for (size_t i = 0; i < algo2_poses.size(); ++i) { // Similarly for algo2
                csv_file << pair_id << "," << algo2_name;
                for (size_t j = 0; j < algo2_poses[i].size(); ++j) {
                    const auto& pose = algo2_poses[i][j];
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

    // Function to save metrics to CSV
    void save_metrics_to_csv(const std::string& algorithm, 
                            int pair_id, 
                            long int planning_time, 
                            long int execution_time, 
                            double smoothness, 
                            double path_length) {
        const std::string filename = "src/PlannerGAN/data/metrics_data.csv";
        // const std::string filename = "metrics_data.csv";
        std::ofstream csv_file(filename, std::ios::app); // Open file in append mode

        // Check if the file needs a header
        bool write_header = !std::filesystem::exists(filename) || std::filesystem::file_size(filename) == 0;

        if (csv_file.is_open()) {
            // Write the header if needed
            if (write_header) {
                csv_file << "PairID,Algorithm,PlanningTime_ms,ExecutionTime_ms,Smoothness,PathLength\n";
            }

            // Write the data
            csv_file << pair_id << ","
                    << algorithm << ","
                    << planning_time << ","
                    << execution_time << ","
                    << smoothness << ","
                    << path_length << "\n";

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Metrics data saved to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", filename.c_str());
        }
    }


    // Function to get the last pair_id used, from a CSV file
    int get_last_pair_id_from_csv(const std::string& filename) {

        // Open the file for reading
        std::ifstream file(filename);
        
        // String variables to store the current line being read, and the last non-empty line found
        std::string line;
        std::string last_line;

        // Check if the file was successfully opened
        if (file.is_open()) {
            // Read the file line by line
            while (getline(file, line)) {
                // Check if the line is not empty
                if (!line.empty()) {
                    // Update last_line with the current line
                    last_line = line;
                }
            }
            // Close the file after reading
            file.close();
        }

        // Check if we have found a last line in the file
        if (!last_line.empty()) {
            // Use a stringstream to parse the line
            std::stringstream ss(last_line);
            std::string item;

            // Read the first item from the line (up to the first comma),
            // which is expected to be the pair_id
            getline(ss, item, ',');

            // Convert the extracted item (pair_id) from string to integer and return it
            return std::stoi(item);
        }

        // If the file is empty or doesn't exist, return 0 as the default pair_id
        return 0;
    }

};

int main(int argc, char** argv) {
    // Seed the random number generator with the current time
    srand(time(NULL));

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningNode>(rclcpp::NodeOptions());

    // Run the planning x times with y number of samples
    int n_iterations = 400;
    int n_samples = 1;
    node->start_planning_n_samples(n_iterations, n_samples);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




