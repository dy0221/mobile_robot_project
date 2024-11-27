import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부 (디폴트는 False)
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # 패키지 경로, xacro
    pkg_path = os.path.join(get_package_share_directory("urdf_pkg"))
    xacro_file = os.path.join(pkg_path, "urdf", "diff_drive_robot_urdf.xacro")

    # xacro 파일을 처리하여 robot_description을 생성
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # 파라미터 설정
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    

    #robot state publisher node 실행
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    #fake_state_publisher
    fake_state_publisher = Node(
        package='urdf_pkg',
        executable='fake_state_publisher',
        output='screen',
        parameters=[],
    )

    #state_publisher
    state_publisher = Node(
        package='urdf_pkg',
        executable='state_publisher',
        output='screen',
        parameters=[],
    )

    #test_rpm_publisher 실행
    test_rpm_publisher = Node(
        package='test_pkg',
        executable='test_rpm_publisher',
        output='screen',
        parameters=[],
    )
    
    #test_rpm_publisher 실행
    test_joint2cmd_publisher = Node(
        package='test_pkg',
        executable='test_joint2cmd_publisher',
        output='screen',
        parameters=[],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
            DeclareLaunchArgument(
                'use_ros2_control',
                default_value='true',
                description='Use ros2_control if true'),

            #joint_state_publisher_gui,
            #test_rpm_publisher,
            #test_joint2cmd_publisher,
            #state_publisher,
            robot_state_publisher,
        ]
    )
