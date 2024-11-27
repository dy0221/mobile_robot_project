#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/motor_rpm.hpp"  

class TestJoint2CmdPublisher : public rclcpp::Node
{
public:
    TestJoint2CmdPublisher() : Node("test_joint2cmd_publisher"), left_rpm(0), right_rpm(0)
    {
        //topic 이름 motor_rpm, 큐 깊이 10
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TestJoint2CmdPublisher::publish_message, this));
        
        subscription_ = this->create_subscription<robot_msgs::msg::MotorRpm>(
            "motor_rpm", 10,
            std::bind(&TestJoint2CmdPublisher::listener_callback, this, std::placeholders::_1));
    }
private:
    void publish_message()
    {
        auto message = geometry_msgs::msg::Twist();
        

        double left_vel = left_rpm*rpm_to_rad_s;
        double right_vel = right_rpm*rpm_to_rad_s;

        double linear_velocity = (left_vel+right_vel) / 2;
        double angular_velocity = (right_vel-left_vel) / wheel_separation;

        // 계산된 속도 값을 메시지에 할당
        message.linear.x = linear_velocity;
        message.angular.z = angular_velocity;
    
        // 메시지 게시
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f", message.linear.x, message.angular.z);
    }
    
    void listener_callback(const robot_msgs::msg::MotorRpm::SharedPtr msg)
    {
        left_rpm = msg->left_motor_rpm;
        right_rpm = msg->right_motor_rpm;
        RCLCPP_INFO(this->get_logger(), "Received: Left RPM: %d, Right RPM: %d", msg->left_motor_rpm, msg->right_motor_rpm);
    }

    rclcpp::Subscription<robot_msgs::msg::MotorRpm>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int left_rpm;
    int right_rpm;
    // 휠 간 거리
    double wheel_separation = 0.35;  // 35 cm (예시)
    double rpm_to_rad_s = 2.0 * 3.14159 / 60.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestJoint2CmdPublisher>());
    rclcpp::shutdown();
    return 0;
}