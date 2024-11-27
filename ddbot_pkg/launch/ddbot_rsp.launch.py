import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부 (디폴트는 False)
    use_sim_time = LaunchConfiguration("use_sim_time")
    # ros2 control 사용 여부 (디폴트는 true)
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # 패키지 경로, xacro
    pkg_path = os.path.join(get_package_share_directory("ddbot_pkg"))
    xacro_file = os.path.join(pkg_path, "urdf", "ddbot_urdf.xacro")

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

            robot_state_publisher,
        ]
    )
