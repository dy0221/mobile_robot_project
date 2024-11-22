import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부 (디폴트는 False)
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 패키지 경로, xacro, rviz2 파일 경로 설정
    pkg_path = os.path.join(get_package_share_directory("urdf_pkg"))
    xacro_file = os.path.join(pkg_path, "urdf", "mobile_robot.xacro")
    rviz2_file = os.path.join(pkg_path, "urdf", "mobile_robot.rviz")

    # xacro 파일을 처리하여 robot_description을 생성
    xacro_processed = xacro.process_file(xacro_file)
    robot_description = xacro_processed.toxml()

    # 파라미터 설정
    params = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
    }

    # LaunchDescription 반환 (robot_state_publisher 노드 실행)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            #robot state publisher node 실행
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            #joint state publisher gui node 실행 << 이게 있어야 움직이는 link가 rviz에 잡힘
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            #rviz2
            Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz2_file]
            )
        ]
    )
