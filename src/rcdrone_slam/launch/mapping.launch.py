from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rcdrone_slam')
    rf2o_params = os.path.join(pkg_dir, 'config', 'rf2o_params.yaml')
    slam_params = '/home/t248/tunnel_slam_params.yaml'

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.10', '3.14159', '0', '0', 'base_footprint', 'laser']
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='CLaserOdometry2DNode',
        output='screen',
        parameters=[rf2o_params]
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}]
    )

    slam_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

    slam_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        static_tf_laser,
        rf2o,
        slam_toolbox,
        TimerAction(period=5.0, actions=[slam_configure]),
        TimerAction(period=7.0, actions=[slam_activate]),
    ])
