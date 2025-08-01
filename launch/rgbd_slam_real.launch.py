import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = get_package_share_directory("my_depth_slam")
    xacro_file = os.path.join(pkg_path, "urdf", "base_with_cam.urdf.xacro")
    slam_config = os.path.join(pkg_path, "config", "mapper_params_online_async.yaml")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation clock if true"
        ),

        # 로봇 모델
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}]
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}]
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
                {'base_frame': 'base_footprint'},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'publish_tf_map': True},

            ]
        ),

        # ⬇️ static transform publisher: base_footprint → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'laser']
        ),

        # RViz2 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # odom → base_footprint (항상 0,0,0) 이걸 없애고 imu센서로 추가해야함
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),

    ])
