import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString


def generate_launch_description():
    namespace = ''
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default='turtlebot_gps2.rviz')


    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world_gps.world'
    )

    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example_turtlebot.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    argument_final_position = DeclareLaunchArgument('output_final_position',default_value='false')
    argument_yaml = DeclareLaunchArgument('output_location', default_value='~/dual_ekf_navsat_example_debug.txt')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

     # Add namespace to rviz config file
    namespaced_rviz_config_file = ReplaceString(
        source_file=PathJoinSubstitution([get_package_share_directory('scout_description'),'rviz',
            rviz_config_file]), 
        replacements={'/robot_namespace': ('', namespace)})

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        name='rviz2',
        arguments=['-d', namespaced_rviz_config_file],
        condition=IfCondition(use_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
            ('/move_base_simple/goal', 'move_base_simple/goal')
        ]
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose
        }.items()
    )

    gps_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_broadcaster',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'gps_link']
    )
    
    localization_local = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/local')]  
    )

    localization_global = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_map',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_node = Node(
        package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu', 'imuWithoutCovariance'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           
    )

    odom_noise_node = Node(
        package='robot_localization',  
        executable='odom_covariance_override',  
        name='odom_covariance_override', 
        output='screen', 
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(rviz_node)
    ld.add_action(localization_local)
    ld.add_action(argument_final_position)
    ld.add_action(argument_yaml)
    ld.add_action(navsat_node)
    ld.add_action(localization_global)
    ld.add_action(gps_frame_publisher)
    ld.add_action(odom_noise_node)

    return ld
