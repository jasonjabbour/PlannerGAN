# PlannerGAN
PlannerGAN


# Commands

Outside of Docker Setup
```shell 
mkdir -p PlannerGAN_ws/src
git clone https://github.com/ICube-Robotics/iiwa_ros2.git
git clone https://github.com/jasonjabbour/PlannerGAN.git
```


Inside of the Docker
```shell

# Go back to the root of your workspace
cd /tmp/PlannerGAN_ws  
colcon build --symlink-install

# Source the Overlay
source /tmp/PlannerGAN_ws/install/setup.bash

# Launch the Gazebo Simulation
ros2 launch iiwa_simulation iiwa_gazebo.launch.py
```
