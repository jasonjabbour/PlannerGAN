# PlannerGAN
PlannerGAN


# Commands

Outside of Docker Setup
```shell 
mkdir -p PlannerGAN_ws/src
git clone https://github.com/ICube-Robotics/iiwa_ros2.git
git clone https://github.com/jasonjabbour/PlannerGAN.git
cd PlannerGAN
code .
```

Inside of Docker Each Time you Start Docker
```shell

# Go back to the root of your workspace
cd /tmp/PlannerGAN_ws 

sudo apt-get update
vcs import src < src/iiwa_ros2/iiwa_ros2.repos
rosdep install --ignore-src --from-paths . -y -r

```

Inside of Docker Each Time Before you Run a Script
```shell
cd /tmp/PlannerGAN_ws 

# Run this twice if you see errors (usually only happes the first time)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

source install/setup.bash

source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/tmp/PlannerGAN_ws/src/iiwa_ros2
```

Launch Files to Choose from:
```shell
# Run only the simulation
ros2 launch plannergan_iiwa iiwa_gazebo.launch.py use_sim:="true"

# Run a planning simulation
# FIX THIS
chmod +x /tmp/PlannerGAN_ws/install/plannergan_iiwa/lib/plannergan_iiwa/rrt_planning.py 
ros2 launch plannergan_iiwa iiwa_gazebo_planning.launch.py use_sim:="true"
```

![](captures/gazebo_setup.png)


## Backup Commands

```shell
colcon build --merge-install --packages-up-to plannergan_iiwa

```