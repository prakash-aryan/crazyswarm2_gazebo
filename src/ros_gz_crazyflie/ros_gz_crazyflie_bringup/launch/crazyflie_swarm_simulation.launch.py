import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


DRONE_NAMES = ['cf1', 'cf2', 'cf3', 'cf4']


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load URDF for robot model visualization
    urdf_path = os.path.join(
        pkg_project_gazebo, 'models', 'crazyflie', 'crazyflie.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Launch Gazebo with the swarm world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'crazyflie_swarm_world.sdf -r'
        ])}.items(),
    )

    # ROS-Gazebo bridge (clock + cmd_vel + odom)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_project_bringup, 'config',
                'ros_gz_crazyflie_swarm_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Consolidated TF publisher from odometry
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

    # One robot_state_publisher + control node per drone
    robot_publishers = []
    control_nodes = []
    for name in DRONE_NAMES:
        # Replace base_footprint with the drone's namespaced frame
        drone_urdf = robot_desc.replace(
            'name="base_footprint"',
            f'name="{name}/base_footprint"')
        robot_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{name}',
            namespace=name,
            parameters=[{
                'robot_description': drone_urdf,
                'frame_prefix': f'{name}/',
                'use_sim_time': True,
            }],
        ))
        control_nodes.append(Node(
            package='ros_gz_crazyflie_control',
            executable='control_services',
            name=f'control_{name}',
            output='screen',
            parameters=[
                {'hover_height': 0.5},
                {'robot_prefix': f'/{name}'},
                {'incoming_twist_topic': '/cmd_vel'},
                {'max_ang_z_rate': 0.4},
                {'use_sim_time': True},
            ]
        ))

    # RViz with TF + RobotModel display
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            pkg_project_bringup, 'config', 'swarm_rviz.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        odom_tf,
        rviz,
    ] + robot_publishers + control_nodes)
