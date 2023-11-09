from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the iiwa URDF/XACRO file
    iiwa_description_path = PathJoinSubstitution([
        FindPackageShare("iiwa_description"),
        "urdf",
        "iiwa.urdf.xacro"
    ])

    # Define the path to the Gazebo world file (you need to create or define your own world file)
    gazebo_world_path = PathJoinSubstitution([
        FindPackageShare("iiwa_description"),
        "worlds",
        "iiwa_world.world"
    ])

    # Use xacro to parse URDF file
    urdf_content = Command(['xacro ', iiwa_description_path])

    # Gazebo server launch
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gzserver.launch.py']),
            launch_arguments={'world': gazebo_world_path}.items(),
    )

    # Gazebo client launch
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gzclient.launch.py'])
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'iiwa', '-topic', 'robot_description'],
        output='screen'
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': urdf_content}],
    )

    # Launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=gazebo_world_path,
            description='The Gazebo world to load'
        ),
        gazebo_server_launch,
        gazebo_client_launch,
        robot_state_publisher,
        spawn_entity,
    ])

    return ld