#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/motor_rpm.hpp"  

class TestRpmSubciber : public rclcpp::Node
{
public:
    TestRpmSubciber() : Node("motor_rpm_subscriber")
    {
        subscription_ = this->create_subscription<robot_msgs::msg::MotorRpm>(
            "motor_rpm", 10,
            std::bind(&TestRpmSubciber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const robot_msgs::msg::MotorRpm::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received: Left RPM: %d, Right RPM: %d", msg->left_motor_rpm, msg->right_motor_rpm);
    }

    rclcpp::Subscription<robot_msgs::msg::MotorRpm>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestRpmSubciber>());
    rclcpp::shutdown();
    return 0;
}
