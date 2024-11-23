#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from robot_msgs.msg import MotorRpm


class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_states.position = [0.0, 0.0]  # 초기 위치
        self.joint_states.velocity = [0.0, 0.0]  # 초기 속도

        # motor_rpm subscriber
        self.sub_motor_rpm = self.create_subscription(
            MotorRpm,
            "motor_rpm",  # topic이름
            self.motor_rpm_callback,
            10
        )

        # motor rpm to rad/s conversion factor
        self.rpm_to_rad_s = 2.0 * 3.14159 / 60.0

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)

        # motor RPM storage
        self.left_motor_rpm = 0
        self.right_motor_rpm = 0
    
    def motor_rpm_callback(self, msg: MotorRpm):
        """
        motor_rpm_topic에서 수신한 메시지를 처리합니다.
        """
        self.left_motor_rpm = msg.left_motor_rpm
        self.right_motor_rpm = msg.right_motor_rpm

        self.get_logger().info(
            f"Received RPMs: Left={self.left_motor_rpm}, Right={self.right_motor_rpm}"
        )

    def publish_callback(self):
        """
        주기적으로 joint_states 메시지를 퍼블리싱합니다.
        """
        curr_time = self.get_clock().now()

        # joint states
        self.joint_states.header.stamp = curr_time.to_msg()
        self.joint_states.velocity[0] = self.left_motor_rpm * self.rpm_to_rad_s
        self.joint_states.velocity[1] = self.right_motor_rpm * self.rpm_to_rad_s

        # 위치 갱신 (속도를 기반으로 계산)
        self.joint_states.position[0] += self.joint_states.velocity[0] * 0.1  # 0.1초 간격
        self.joint_states.position[1] += self.joint_states.velocity[1] * 0.1

        # publish
        self.pub_joint_states.publish(self.joint_states)


def main(args=None):
    rclpy.init(args=args)

    driver = StatePublisher()
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()