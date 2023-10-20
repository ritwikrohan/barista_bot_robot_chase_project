import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
   # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Define the robot model files to be used
    robot_desc_file = "barista_robot_model.urdf"
    robot_desc_path = os.path.join(get_package_share_directory(
        "barista_robot_description"), "urdf", robot_desc_file)


    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'barista_robot', '-x', '0.0', '-y', '0.0', '-z', '0.2',
                   '-topic', '/robot_description']
    )


    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory("barista_robot_description"), 'rviz', 'config.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
        rviz_node
    ])