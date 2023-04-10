#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
// geometry_msgs/msg/Twist

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
        : Node("cmd_vel_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_unstamped", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&CmdVelPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 2.0;
        message.angular.z = 1.8;
        count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: LinearX => %lf, AngularZ => %lf", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}