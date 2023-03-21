#include <memory>
#include <math.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "FileStream.h"

#define waypointsFilePath "/home/h/waypoints.dat"
#define fomattedWaypointsFilePath "/home/h/waypoints_formatted.dat"


class OrientationLogger : public rclcpp::Node
{
public:
    OrientationLogger() : Node("orientation_logger")
    {
        auto qos_be = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 10), rmw_qos_profile_sensor_data);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                        "poselogger", 10, std::bind(&OrientationLogger::topic_callback, this, std::placeholders::_1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                        "odom", qos_be, std::bind(&OrientationLogger::callback_odom, this, std::placeholders::_1));
        sub_writeOdom = this->create_subscription<std_msgs::msg::Empty>(
                        "writeOdom", 10, std::bind(&OrientationLogger::callback_writeOdom, this, std::placeholders::_1));
        sub_camera = this->create_subscription<sensor_msgs::msg::Image>(
                        "color/preview/image", 10, std::bind(&OrientationLogger::callback_camera, this, std::placeholders::_1));
        
        fs = new FileStream(waypointsFilePath);
        fs_formatted = new FileStream(fomattedWaypointsFilePath);
        
        RCLCPP_INFO(this->get_logger(), "Pose logger launched!");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_writeOdom;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera;
    FileStream *fs = nullptr;
    FileStream *fs_formatted = nullptr;
    nav_msgs::msg::Odometry currentOdom;

    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
    {
        float q[4];
        q[0] = msg->pose.pose.orientation.w;
        q[1] = msg->pose.pose.orientation.x;
        q[2] = msg->pose.pose.orientation.y;
        q[3] = msg->pose.pose.orientation.z;
        float theta = atan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (pow(q[2], 2) + pow(q[3],2)));
        theta *= M_1_PI * 180;

        std::string poseString = std::to_string(msg->pose.pose.position.x) + "_" + std::to_string(msg->pose.pose.position.y) + "_" + std::to_string(theta) + "\n";
        
        fs->writeToFile(poseString, true);
        fs_formatted->writeToFile_formatted(msg->pose.pose.position.x, msg->pose.pose.position.y, theta);

        RCLCPP_INFO(this->get_logger(), "Writing pose: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y, theta);
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        currentOdom = *msg;
    }

    void callback_writeOdom(const std_msgs::msg::Empty::SharedPtr msg)
    {
        float q[4];
        q[0] = currentOdom.pose.pose.orientation.w;
        q[1] = currentOdom.pose.pose.orientation.x;
        q[2] = currentOdom.pose.pose.orientation.y;
        q[3] = currentOdom.pose.pose.orientation.z;
        float theta = atan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (pow(q[2], 2) + pow(q[3],2)));
        theta *= M_1_PI * 180;

        std::string poseString = std::to_string(currentOdom.pose.pose.position.x) + "_" + std::to_string(currentOdom.pose.pose.position.y) + "_" + std::to_string(theta) + "\n";
        
        fs->writeToFile(poseString, true);
        fs_formatted->writeToFile_formatted(currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y, theta);

        RCLCPP_INFO(this->get_logger(), "Writing pose: %f, %f, %f", currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y, theta);
    }

    void callback_camera(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // for(auto e : msg->data)
        //     printf("%c\n", e);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientationLogger>());
    rclcpp::shutdown();
    return 0;
}
