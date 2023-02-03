#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <stdio.h>


class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher")
    {
        subscription_odom = create_subscription<nav_msgs::msg::Odometry>(
                            "odom", qos_bestEffort, std::bind(&OdomPublisher::callback_odom, this, std::placeholders::_1));
        publisher_odom = create_publisher<nav_msgs::msg::Odometry>("odom_reliable", qos_reliable);

        printf("Odom publisher launched!\n");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;

    rclcpp::QoS qos_bestEffort = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    rclcpp::QoS qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);


    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        publisher_odom->publish(*msg);
        printf("Odom msg relayed.\n");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
