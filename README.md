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

# Install packages for iiwa_ros2
vcs import src < src/iiwa_ros2/iiwa_ros2.repos
rosdep install --ignore-src --from-paths . -y -r

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Source the Overlay
source /tmp/PlannerGAN_ws/install/setup.bash

# Launch the Gazebo Simulation
ros2 launch iiwa_simulation iiwa_gazebo.launch.py
```
