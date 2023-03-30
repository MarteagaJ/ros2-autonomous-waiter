#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class OdometryTwistSubscriber : public rclcpp::Node
{
  public:
    OdometryTwistSubscriber()
    : Node("odometry_twist_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "odometry", 10, std::bind(&OdometryTwistSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: LinearX => %lf, AngularZ => %lf", msg.linear.x, msg.angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTwistSubscriber>());
  rclcpp::shutdown();
  return 0;
}