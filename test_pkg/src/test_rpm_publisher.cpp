#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/motor_rpm.hpp"

class TestRpmPublisher : public rclcpp::Node
{
public:
    TestRpmPublisher() : Node("test_rpm_publisher"), left_rpm(0), right_rpm(0)
    {
        //topic 이름 motor_rpm, 큐 깊이 10
        publisher_ = this->create_publisher<robot_msgs::msg::MotorRpm>("motor_rpm", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TestRpmPublisher::publish_message, this));
    }
private:
    void publish_message()
    {
        auto message = robot_msgs::msg::MotorRpm();
        message.left_motor_rpm = 20; //left_rpm
        message.right_motor_rpm = 20; //right_rpm
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: Left RPM: %d, Right RPM: %d", message.left_motor_rpm, message.right_motor_rpm);
    }

    rclcpp::Publisher<robot_msgs::msg::MotorRpm>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int left_rpm;
    int right_rpm;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestRpmPublisher>());
    rclcpp::shutdown();
    return 0;
}