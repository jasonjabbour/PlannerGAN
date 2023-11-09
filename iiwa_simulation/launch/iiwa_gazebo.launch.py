from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the empty Gazebo world file
    empty_world_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'worlds',
        'empty.world'
    ])
    
    # Define the path to the iiwa XACRO file
    iiwa_xacro_path = PathJoinSubstitution([
        FindPackageShare("iiwa_description"),
        'urdf',
        'iiwa.urdf.xacro'
    ])
    
    # Command to convert XACRO to URDF
    urdf_content = Command(['xacro ', iiwa_xacro_path])

    # Gazebo server launch
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': empty_world_path}.items(),
    )

    # Gazebo client launch
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': urdf_content}],
    )

    # Define launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=empty_world_path,
            description='The Gazebo world to load'
        ),
        gazebo_server_launch,
        gazebo_client_launch,
        robot_state_publisher
    ])