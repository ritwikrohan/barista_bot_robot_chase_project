import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    install_dir = get_package_prefix("barista_robot_description")
    gazebo_models_path = os.path.join(get_package_share_directory("barista_robot_description"), 'meshes')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

   # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Define the robot model files to be used
    robot_desc_file = "barista_robot_model_rick_morty.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory(
        "barista_robot_description"), "xacro", robot_desc_file)

     # convert XACRO file into URDF
    # doc = xacro.parse(open(robot_desc_path))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml(), 'use_sim_time': True}
    
    robot_name_1 = "rick"
    robot_name_2 = "morty"

    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1])}],
        output="screen"
    )

    rsp_robot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_2,
        parameters=[{'frame_prefix': robot_name_2+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_2])}],
        output="screen"
    )

    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-x', '0.0', '-y', '0.0', '-z', '0.2',
                   '-topic', robot_name_1+'/robot_description']
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot2', '-x', '-1.0', '-y', '-1.0', '-z', '0.2',
                   '-topic', robot_name_2+'/robot_description']
    )

    # rsp = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[params],
    #     output="screen"
    # )

    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'barista_robot', '-x', '0.0', '-y', '0.0', '-z', '0.2',
    #                '-topic', '/robot_description']
    # )


    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory("barista_robot_description"), 'rviz', 'config_cp8.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    static_tf_pub_rick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_rick_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'rick/odom']
    )

    static_tf_pub_morty = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_morty_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'morty/odom']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(get_package_share_directory("barista_robot_description"),'worlds', 'empty.world'), ''],
          description='SDF world file'),
        gazebo,
        rsp_robot1,
        rsp_robot2,
        spawn_robot1,
        spawn_robot2,
        rviz_node,
        static_tf_pub_rick,
        static_tf_pub_morty
    ])