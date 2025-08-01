#!/usr/bin/env python3
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
        # ───── launch 인자 ─────
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation clock if true"
        ),

        # ───── 로봇 모델 퍼블리셔 ─────
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

        # ───── SLAM Toolbox ─────
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
                {'publish_pose': True}, 
            ]
        ),

        # ▶ RF2O Laser Odom
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom_rf2o',
                'publish_tf' : True,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 30.0}],
        ),
        #───── RealSense D435i IMU Only ─────
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="d435i",
            output="screen",
            parameters=[{
                "camera_name": "d435i",
                "device_type": "d435i",
                "enable_accel": True,
                "enable_gyro": True,
                "enable_depth": False,
                "enable_color": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "unite_imu_method": 1
            }],
            arguments=["--ros-args", "--log-level", "error"],
            emulate_tty=True
        ),  
    
        #───── IMU Madgwick Filter (camera IMU → filtered) ─────
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[{
                "use_mag": False,
                "publish_tf": False,
                "fixed_frame": "base_footprint"
            }],
            remappings=[
                ("imu/data_raw", "/camera/d435i/imu")
            ]
        ),
        # ───── SL-LiDAR A2M12 노드 ─────
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_a2m12",
            output="screen",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",  # 실제 연결된 포트 확인해서 수정 가능
                "serial_baudrate": 256000,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True
            }]
        ),


        
        # ───── EKF (IMU → odom 추정) ─────
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                "/home/hs/ros_ws/src/my_depth_slam/config/ekf_imu.yaml",
                {"use_sim_time": use_sim_time}
            ]
        ),
        
        # ───── static transform: base_footprint → laser ─────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'laser']
        ),

        # ───── RViz2 ─────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
