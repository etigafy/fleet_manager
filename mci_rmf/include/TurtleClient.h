#pragma once

#include <stdio.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "CustomTypes.h"

#include "h_interfaces/msg/client_state.hpp"
#include "h_interfaces/msg/client_task.hpp"
#include "h_interfaces/srv/elevator_pos.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "irobot_create_msgs/msg/dock_status.hpp"s
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"

#include "action_interfaces/action/dock.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;
using namespace h_interfaces::msg;
using namespace h_interfaces::srv;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace nav2_msgs::action;
using namespace irobot_create_msgs::msg;
using namespace irobot_create_msgs::action;
using namespace sensor_msgs::msg;


// ------------------------------------------------------------------------------------------------
class TurtleClient : public rclcpp::Node
{
public:
    TurtleClient() : Node("turtleClient")
    {

        // callback groups
        // rclcpp::Call cg_clientState = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // init parameters
        declare_parameter("clientID", 9999);
        clientID = get_parameter("clientID").get_parameter_value().get<int>();

        // init timers
        timer_clientState = create_wall_timer(500ms, std::bind(&TurtleClient::callback_clientState, this));

        // init subscriptions
        rclcpp::SubscriptionOptions options_clientTask;
        options_clientTask.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto qos_sensor_data = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        subscription_clientTask = create_subscription<ClientTask>("clientTask", 10, std::bind(&TurtleClient::callback_clientTask, this, std::placeholders::_1), options_clientTask);
        subscription_dock = create_subscription<Dock>("dock", qos_sensor_data, std::bind(&TurtleClient::callback_dock, this, std::placeholders::_1));
        subscription_batteryState = create_subscription<BatteryState>("battery_state", qos_sensor_data, std::bind(&TurtleClient::callback_batteryState, this, std::placeholders::_1));
        subscription_odom = create_subscription<Odometry>("odom", qos_sensor_data, std::bind(&TurtleClient::callback_odom, this, std::placeholders::_1));

        // init publishers
        publisher_clientState = create_publisher<ClientState>("clientState", 10);
        
        // init services
        service_elevatorPos = create_client<ElevatorPos>("elevator_service");

        // init actions
        action_dock = rclcpp_action::create_client<Dock>(this, "dock");
        action_undock = rclcpp_action::create_client<Undock>(this, "undock");
        action_navigateToPose = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        action_cameraDocking = rclcpp_action::create_client<action_interfaces::action::Dock>(this, "dock_turtle");
        action_createMoveLinear = rclcpp_action::create_client<DriveDistance>(this, "drive_distance");

        // finish setup
        RCLCPP_INFO(get_logger(), "TurtleClient [TT%d] launched.", clientID);
        RCLCPP_INFO(get_logger(), "--------------------");

        // moveLinear(0.2);
        // moveElevator(150);
    }

private:
    int clientID;
    float batteryPercent = 1.0;
    PoseStamped clientPose = PoseStamped();
    bool taskFinished = false;
    int isDocked = -1;
    bool isNavigating = false;
    bool isCameraDocking = false;
    bool isMoveLinear = false;
    int counter = 0;

    // timers
    rclcpp::TimerBase::SharedPtr timer_clientState;

    // subscriptions
    rclcpp::Subscription<ClientTask>::SharedPtr subscription_clientTask;
    rclcpp::Subscription<Dock>::SharedPtr subscription_dock;
    rclcpp::Subscription<BatteryState>::SharedPtr subscription_batteryState;
    rclcpp::Subscription<Odometry>::SharedPtr subscription_odom;

    // publishers
    rclcpp::Publisher<ClientState>::SharedPtr publisher_clientState;

    // services
    rclcpp::Client<ElevatorPos>::SharedPtr service_elevatorPos;

    // actions
    rclcpp_action::Client<Dock>::SharedPtr action_dock;
    rclcpp_action::Client<Undock>::SharedPtr action_undock;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_navigateToPose;
    rclcpp_action::Client<action_interfaces::action::Dock>::SharedPtr action_cameraDocking;
    rclcpp_action::Client<DriveDistance>::SharedPtr action_createMoveLinear;
    // callbacks
    void callback_clientState();
    void callback_dock(Dock msg);
    void callback_clientTask(ClientTask msg);
    void callback_navigateToPose_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future);
    void callback_navigateToPose_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result);
    void callback_batteryState(BatteryState msg);
    void callback_odom(Odometry msg);
    void callback_cameraDocking_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<action_interfaces::action::Dock>::SharedPtr> future);
    void callback_cameraDocking_result(const rclcpp_action::ClientGoalHandle<action_interfaces::action::Dock>::WrappedResult &result);
    void callback_createMoveLinear_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<DriveDistance>::SharedPtr> future);
    void callback_createMoveLinear_result(const rclcpp_action::ClientGoalHandle<DriveDistance>::WrappedResult &result);

    void callback_clientTask1(ClientTask msg);

    // utility
    void task_idle(ClientTask msg);
    void task_charge(ClientTask msg);
    void task_pickup(ClientTask msg);
    void task_dropoff(ClientTask msg);
    float quaternion_2_yaw(geometry_msgs::msg::Quaternion q);
    void moveToPose(PoseStamped goalPose);
    void cameraDock();
    void dock();
    void undock();
    bool getDockedStatus();

    // elevator
    void moveElevator(float pos);

    // movement
    void moveLinear(float distance);
};
