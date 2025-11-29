from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('control2')  # Change to your package name
    
    # Path to configuration files
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')
    
    return LaunchDescription([
        
        # Local EKF - fuses continuous sensors (IMUs + Visual Odometry) in odom frame
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_local_config],
            remappings=[
                ('odometry/filtered', 'odometry/local')
            ]
        ),
        
        # navsat_transform - converts GPS lat/lon to odometry in map frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('imu/data', '/zed2i/imu/data'),  # Use ZED2i IMU for heading
                ('gps/fix', '/gps/fix'),  # Your GPS topic
                ('odometry/filtered', 'odometry/local'),  # Input from local EKF
                ('odometry/gps', 'odometry/gps')  # Output GPS-based odometry
            ]
        ),
        
        # Global EKF - fuses everything including GPS in map frame
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_global_config],
            remappings=[
                ('odometry/filtered', 'odometry/global')
            ]
        ),
    ])
