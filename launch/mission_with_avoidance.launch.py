from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'drone_lidar_avoidance_py'

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='mission_params.yaml',
        description='Mission YAML file located in the config folder'
    )

    offboard_file_arg = DeclareLaunchArgument(
        'offboard_file',
        default_value='offboard_params.yaml',
        description='Offboard YAML file located in the config folder'
    )

    gz_scan_topic_arg = DeclareLaunchArgument(
        'gz_scan_topic',
        default_value='/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
        description='Gazebo LiDAR scan topic to bridge into ROS 2'
    )

    obstacle_distance_threshold_arg = DeclareLaunchArgument(
        'obstacle_distance_threshold',
        default_value='4.0',
        description='Front obstacle detection threshold in meters'
    )

    mission_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        LaunchConfiguration('mission_file')
    ])

    offboard_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        LaunchConfiguration('offboard_file')
    ])

    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            [
                LaunchConfiguration('gz_scan_topic'),
                '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ]
        ]
    )

    offboard_control_node = Node(
        package=package_name,
        executable='offboard_control',
        name='offboard_control_node',
        output='screen',
        parameters=[offboard_params_file]
    )

    mission_manager_node = Node(
        package=package_name,
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
        parameters=[mission_params_file]
    )

    lidar_processor_node = Node(
        package=package_name,
        executable='lidar_processor',
        name='lidar_processor_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('gz_scan_topic'),
            'front_angle_deg': 20.0,
            'side_min_angle_deg': 20.0,
            'side_max_angle_deg': 100.0,
            'log_rate_hz': 1.0,
        }]
    )

    obstacle_detector_node = Node(
        package=package_name,
        executable='obstacle_detector',
        name='obstacle_detector_node',
        output='screen',
        parameters=[{
            'obstacle_distance_threshold': LaunchConfiguration('obstacle_distance_threshold'),
            'log_rate_hz': 20.0,
        }]
    )

    avoidance_manager_node = Node(
        package=package_name,
        executable='avoidance_manager',
        name='avoidance_manager_node',
        output='screen',
        parameters=[{
            'avoidance_offset': 4.0,
            'goal_tolerance': 0.5,
            'timer_rate_hz': 20.0,
        }]
    )

    return LaunchDescription([
        mission_file_arg,
        offboard_file_arg,
        gz_scan_topic_arg,
        obstacle_distance_threshold_arg,
        lidar_bridge_node,
        offboard_control_node,
        mission_manager_node,
        lidar_processor_node,
        obstacle_detector_node,
        avoidance_manager_node,
    ])