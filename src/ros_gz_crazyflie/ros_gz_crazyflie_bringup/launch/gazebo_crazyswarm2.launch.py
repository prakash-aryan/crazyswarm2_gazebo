"""
Gazebo swarm world + Crazyswarm2 integration launch.

Starts Gazebo with 4 Crazyflies, the ROS-GZ bridge, TF publisher,
robot models, and Crazyswarm2 server with the gazebo backend.
No control_services nodes — Crazyswarm2 handles all flight control.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


DRONE_NAMES = ['cf1', 'cf3', 'cf4']


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gazebo, 'worlds', 'crazyflie_swarm_world.sdf -r'
        ])}.items(),
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_bringup, 'config',
                'ros_gz_crazyflie_swarm_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # TF publisher from odometry
    odom_tf = Node(
        package='ros_gz_crazyflie_control',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen',
        parameters=[
            {'drone_names': DRONE_NAMES},
            {'use_sim_time': True},
        ]
    )

    # Crazyswarm2 publishes its own robot_description per drone.
    # The odom_tf_publisher adds cfX/base_footprint -> cfX identity
    # transforms so the descriptions find their TF frames.

    # Crazyswarm2 sim server directly (with use_sim_time for Gazebo clock sync)
    crazyswarm2_yaml = os.path.join(
        get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml')
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'), 'config', 'server.yaml')

    import yaml
    with open(crazyswarm2_yaml, 'r') as f:
        cf_params = yaml.safe_load(f)
    with open(server_yaml, 'r') as f:
        srv_params = yaml.safe_load(f)['/crazyflie_server']['ros__parameters']

    urdf_cs2 = os.path.join(
        get_package_share_directory('crazyflie'), 'urdf', 'crazyflie_description.urdf')
    with open(urdf_cs2, 'r') as f:
        srv_params['robot_description'] = f.read()

    crazyswarm2_server = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        emulate_tty=True,
        parameters=[cf_params, srv_params, {'use_sim_time': True}],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'swarm_rviz.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        odom_tf,
        rviz,
        crazyswarm2_server,
    ])
