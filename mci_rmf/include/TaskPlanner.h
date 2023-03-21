#pragma once

#include <stdio.h>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "CustomTypes.h"
#include "Task.h"
#include "Client.h"
#include "Landmark.h"
#include "std_srvs/srv/empty.hpp"
#include "h_interfaces/msg/client_state.hpp"
#include "h_interfaces/msg/client_task.hpp"
#include "h_interfaces/msg/status.hpp"
#include "h_interfaces/msg/client_object.hpp"
#include "h_interfaces/msg/delivery_object.hpp"
#include "h_interfaces/srv/delivery_request.hpp"
#include "h_interfaces/srv/task_request.hpp"
#include "h_interfaces/srv/planner_api.hpp"
#include "gui_interfaces/srv/gui.hpp"

using namespace std::chrono_literals;
using namespace h_interfaces::msg;
using namespace h_interfaces::srv;
using namespace gui_interfaces::srv;


// ------------------------------------------------------------------------------------------------
class TaskPlanner : public rclcpp::Node
{
public:
    TaskPlanner() : Node("TaskPlanner")
    {
        // load landmarks
        if(loadLandmarks() == 0) printf("Landmarks loaded.\n");

        // init timers
        timer_clientTimeout = create_wall_timer(2500ms, std::bind(&TaskPlanner::callback_clientTimeout, this));
        timer_status = create_wall_timer(1000ms, std::bind(&TaskPlanner::callback_status, this));
        timer_clientInterface = create_wall_timer(500ms, std::bind(&TaskPlanner::callback_clientInterface, this));
        timer_taskDistributor = create_wall_timer(500ms, std::bind(&TaskPlanner::callback_taskDistributor, this));

        // init subscriptions
        subscription_clientState = create_subscription<ClientState>(
            "clientState", 10, std::bind(&TaskPlanner::callback_clientState, this, std::placeholders::_1));

        // init publishers
        publisher_status = create_publisher<Status>("status", 10);
        publisher_clientTask = create_publisher<ClientTask>("clientTask", 10);
        
        // init services
        service_packageRequest = create_service<Gui>("GuiService", 
            std::bind(&TaskPlanner::callback_deliveryRequest, this, std::placeholders::_1, std::placeholders::_2));
        service_statusRequest = create_service<PlannerApi>("statusRequest",
            std::bind(&TaskPlanner::callback_statusRequest, this, std::placeholders::_1, std::placeholders::_2));

        // finish setup
        printf("TaskPlanner started.\n");
        printf("--------------------\n");
    }

private:
    std::map<int, Client*> clientMap;
    std::map<LandmarkType, std::map<int, Landmark*>> landmarkMap;
    std::map<int, std::map<int, TaskGroup*>> deliveryMap;
    int deliveryCounter = 1;

    // timers
    rclcpp::TimerBase::SharedPtr timer_clientTimeout;
    rclcpp::TimerBase::SharedPtr timer_status;
    rclcpp::TimerBase::SharedPtr timer_clientInterface;
    rclcpp::TimerBase::SharedPtr timer_taskDistributor;

    // subscriptions
    rclcpp::Subscription<ClientState>::SharedPtr subscription_clientState;

    // publishers
    rclcpp::Publisher<Status>::SharedPtr publisher_status;
    rclcpp::Publisher<ClientTask>::SharedPtr publisher_clientTask;

    // services
    rclcpp::Service<Gui>::SharedPtr service_packageRequest;
    rclcpp::Service<PlannerApi>::SharedPtr service_statusRequest;

    // callbacks
    void callback_clientState(const ClientState msg);
    void callback_clientTimeout();
    void callback_status();
    void callback_clientInterface();
    void callback_taskDistributor();
    void callback_deliveryRequest(const std::shared_ptr<Gui::Request> request, std::shared_ptr<Gui::Response> response);
    void callback_statusRequest(const std::shared_ptr<PlannerApi::Request> request, std::shared_ptr<PlannerApi::Response> response);

    // utility
    int loadLandmarks();
    void updateClientData(const ClientState msg, Client* client);
    void publishMsg_ClientTask(Client* client, TaskType taskType);
    float quaternion_2_yaw(geometry_msgs::msg::Quaternion q);
    int bookLandmark(Client* client, LandmarkType landmark_type, int landmark_id=-1);
    int chooseNewTaskGroup(int* deliveryID, int* taskGroupID);
};