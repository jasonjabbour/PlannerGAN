from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the iiwa_description package
    iiwa_description_pkg_path = get_package_share_directory('iiwa_description')

    # Set the path to the iiwa URDF/Xacro file
    iiwa_urdf_file = os.path.join(iiwa_description_pkg_path, 'urdf', 'iiwa.urdf.xacro')

    # Set the path to the Gazebo launch file
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # Include Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file]),
            launch_arguments={'world': os.path.join(iiwa_description_pkg_path, 'gazebo', 'worlds', 'empty.world')}.items(),
        ),

        # Spawn iiwa URDF/Xacro
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'iiwa', '-topic', 'robot_description'],
            output='screen',
        ),
    ])