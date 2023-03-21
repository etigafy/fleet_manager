#include "TurtleClient.h"
#include "CustomTypes.h"

#include <thread>

#define ELEVATOR_TOP_DROPOFF 146.5 // mm
#define ELEVATOR_TOP_PICKUP 20 // mm
#define ELEVATOR_BOTTOM 0 // mm
#define CAMERA_UNDOCK_DISTANCE -0.25 // m
#define DELAY_DROPOFF 1000 // ms
#define DELAY_PICKUP 1000 // ms

void TurtleClient::callback_clientTask1(ClientTask msg)
{
    RCLCPP_INFO(get_logger(), "sleeping for 2.5s...");
    std::this_thread::sleep_for(2500ms);
    int temp = msg.clientid;

    // report finished task
    taskFinished = true;
    RCLCPP_INFO(get_logger(), "Task finished! %d", temp);
    RCLCPP_INFO(get_logger(), "-------------------------");
}

void TurtleClient::callback_clientTask(ClientTask msg)
{
    // compare IDs
    if(msg.clientid != clientID) return;

    TaskType tt = static_cast<TaskType>(msg.tasktype);
    RCLCPP_INFO(get_logger(), "New Task: %s", CustomTypes::getString(tt));

    // assign tasks
    switch (msg.tasktype)
    {
    case TASK_IDLE:
        task_idle(msg);
        break;

    case TASK_CHARGE:
        task_charge(msg);
        break;

    case TASK_PICKUP:
        task_pickup(msg);
        break;

    case TASK_DROPOFF:
        task_dropoff(msg);
        break;

    default:
        RCLCPP_INFO(get_logger(), "ERROR - Unknown taskType!");
        break;
    }

    // report finished task
    taskFinished = true;
    RCLCPP_INFO(get_logger(), "Task finished!");
    RCLCPP_INFO(get_logger(), "-------------------------");
}

// ------------------------------------- Tasks -----------------------------------------------------------

void TurtleClient::task_idle(ClientTask msg)
{
    moveToPose(msg.pose);
}

void TurtleClient::task_charge(ClientTask msg)
{
    // do nothing if client is already charging
    if(getDockedStatus()) return;

    // move to charging station
    moveToPose(msg.pose);

    // dock to station
    dock();
}

void TurtleClient::task_pickup(ClientTask msg)
{
    // move to pickup pos
    moveToPose(msg.pose);

    // use camera to dock
    // RCLCPP_INFO(get_logger(), "CAMERA DOCK HERE");
    // cameraDock();

    // move elevator up
    // moveElevator(ELEVATOR_TOP_PICKUP);

    // wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_PICKUP));

    // move elevator down
    // moveElevator(ELEVATOR_BOTTOM);

    // camera undock
    moveLinear(CAMERA_UNDOCK_DISTANCE);
}

void TurtleClient::task_dropoff(ClientTask msg)
{
    // move to dropoff pos
    moveToPose(msg.pose);

    // use camera to dock
    // RCLCPP_INFO(get_logger(), "CAMERA DOCK HERE");
    // cameraDock();

    // move elevator up
    moveElevator(ELEVATOR_TOP_DROPOFF);

    // wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_DROPOFF));

    // move elevator down
    moveElevator(ELEVATOR_BOTTOM);

    // camera undock
    moveLinear(CAMERA_UNDOCK_DISTANCE);
}

// ----------------------------------- moveToPose -------------------------------------------------------------
void TurtleClient::moveToPose(PoseStamped goalPose)
{
    // make sure client is not docked
    undock();

    goalPose.header.frame_id = "map";

    // set nav flag
    isNavigating = true;

    RCLCPP_INFO(get_logger(), "[nav2] Waiting for server...");

    if( !action_navigateToPose->wait_for_action_server(1s) ) rclcpp::shutdown();

    RCLCPP_INFO(get_logger(), "[nav2] Sending goal: %f, %f", goalPose.pose.position.x, goalPose.pose.position.y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goalPose;
    goal_msg.behavior_tree = "";

    // set callbacks
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TurtleClient::callback_navigateToPose_goal_response, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&TurtleClient::callback_navigateToPose_result, this, std::placeholders::_1);

    // send goal
    auto future = action_navigateToPose->async_send_goal(goal_msg, send_goal_options);
    
    // wait for future to complete
    size_t counter = 0;
    while(isNavigating)
    {
        if((counter % 25) == 0)
        RCLCPP_INFO(get_logger(), "[nav2] navigating...");
        std::this_thread::sleep_for(100ms);
        counter++;
    }
}

// ----------------------------------- cameraDocking -------------------------------------------------------------
void TurtleClient::cameraDock()
{
    // set cameraDocking flag
    isCameraDocking = true;

    RCLCPP_INFO(get_logger(), "[CD] Waiting for server...");

    if( !action_cameraDocking->wait_for_action_server(1s) ) rclcpp::shutdown();

    RCLCPP_INFO(get_logger(), "[CD] Sending goal");

    auto goal_msg = action_interfaces::action::Dock::Goal();

    // set callbacks
    auto send_goal_options = rclcpp_action::Client<action_interfaces::action::Dock>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TurtleClient::callback_cameraDocking_goal_response, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&TurtleClient::callback_cameraDocking_result, this, std::placeholders::_1);

    // send goal
    auto future = action_cameraDocking->async_send_goal(goal_msg, send_goal_options);
    
    // wait for future to complete
    size_t counter = 0;
    while(isCameraDocking)
    {
        if((counter % 25) == 0)
            RCLCPP_INFO(get_logger(), "[CD] Camera docking...");
        std::this_thread::sleep_for(100ms);
        counter++;
    }
}

// ----------------------------------- docking -------------------------------------------------------------
bool TurtleClient::getDockedStatus()
{
    while(isDocked < 0)
    {
        std::this_thread::sleep_for(100ms);
    }
    // RCLCPP_INFO(get_logger(), "[isDocked] %d", isDocked);

    return static_cast<bool>(isDocked);
}

void TurtleClient::dock()
{
    // return if robot is docked
    if( getDockedStatus() ) return;

    RCLCPP_INFO(get_logger(), "Waiting for dock server...");
    if( !action_dock->wait_for_action_server() ) rclcpp::shutdown();

    auto goal_msg = Dock::Goal();
    auto future = action_dock->async_send_goal(goal_msg);

    // wait for undock to complete
    size_t counter = 0;
    while( !getDockedStatus() )
    {
        if((counter % 10) == 0)
        RCLCPP_INFO(get_logger(), "[Dock] Docking...");
        std::this_thread::sleep_for(100ms);
        counter++;
    }

}

void TurtleClient::undock()
{
    // return if robot is not docked
    if( !getDockedStatus() ) return;

    RCLCPP_INFO(get_logger(), "Waiting for undock server...");
    if( !action_undock->wait_for_action_server() ) rclcpp::shutdown();

    auto goal_msg = Undock::Goal();
    auto future = action_undock->async_send_goal(goal_msg);

    // wait for undock to complete
    size_t counter = 0;
    while(getDockedStatus())
    {
        if((counter % 10) == 0)
        RCLCPP_INFO(get_logger(), "[Dock] Undocking...");
        std::this_thread::sleep_for(100ms);
        counter++;
    }
}

// ----------------------------------- elevator -------------------------------------------------------------
void TurtleClient::moveElevator(float pos)
{
    // create request
    auto request = std::make_shared<h_interfaces::srv::ElevatorPos::Request>();

    request->position = pos;
    request->velocity = 16;
    request->acceleration = 16;

    // wait for service to be available
    while (!service_elevatorPos->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "[elevator] Interrupted while waiting for the service. Exiting.");
        return;
        }
        RCLCPP_INFO(get_logger(), "[elevator] elevator_service not available, waiting...");
    }

    RCLCPP_INFO(get_logger(), "-> Sending elevator request: %.1fmm", pos);

    // send request
    auto result = service_elevatorPos->async_send_request(request);
    // bool isMovingElevator = result.get()->status;

    // wait for service to complete
    int waitTime = ELEVATOR_TOP_DROPOFF / request->velocity + 2;
    for(int i = 0; i < waitTime; i++)
    {
        RCLCPP_INFO(get_logger(), "[elevator] Moving platform... %ds remaining!", waitTime - i);
        std::this_thread::sleep_for(1000ms);
    }
}

// ----------------------------------- moveLinear -------------------------------------------------------------
void TurtleClient::moveLinear(float distance)
{
    // set moveLinear flag
    isMoveLinear = true;

    RCLCPP_INFO(get_logger(), "[ML] Waiting for server...");

    if( !action_createMoveLinear->wait_for_action_server(1s) ) rclcpp::shutdown();

    RCLCPP_INFO(get_logger(), "[ML] Sending goal");

    auto goal_msg = DriveDistance::Goal();
    goal_msg.distance = distance;
    goal_msg.max_translation_speed = 0.1;

    // set callbacks
    auto send_goal_options = rclcpp_action::Client<DriveDistance>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TurtleClient::callback_createMoveLinear_goal_response, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&TurtleClient::callback_createMoveLinear_result, this, std::placeholders::_1);

    // send goal
    action_createMoveLinear->async_send_goal(goal_msg, send_goal_options);
    int waitTime = goal_msg.distance / goal_msg.max_translation_speed; // s
    std::this_thread::sleep_for(std::chrono::seconds(waitTime));
    
    // // wait for future to complete
    // size_t counter = 0;
    // while(isMoveLinear)
    // {
    //     if((counter % 25) == 0)
    //         RCLCPP_INFO(get_logger(), "[ML] Translating...");
    //     std::this_thread::sleep_for(100ms);
    //     counter++;
    // }
}

// ---------------------------- NavigateToPose action callbacks --------------------------------------------------------------------

void TurtleClient::callback_navigateToPose_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
{
    auto goal_handle = future.get();
    if(!goal_handle)
    {
        RCLCPP_INFO(get_logger(), "[nav2] NavigateToPose: goal rejected!");
        isNavigating = false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "[nav2] NavigateToPose: goal accepted!");
    }
}

void TurtleClient::callback_navigateToPose_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "[nav2] Goal succeeded");
            isNavigating = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "[nav2] Goal was aborted");
            isNavigating = false;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "[nav2] Goal was canceled");
            isNavigating = false;
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "[nav2] Unknown result code");
            isNavigating = false;
            return;
    }
}

// ---------------------------- cameraDocking action callbacks --------------------------------------------------------------------

void TurtleClient::callback_cameraDocking_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<action_interfaces::action::Dock>::SharedPtr> future)
{
    auto goal_handle = future.get();
    if(!goal_handle)
    {
        RCLCPP_INFO(get_logger(), "[CD] CameraDocking: goal rejected!");
        isCameraDocking = false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "[CD] CameraDocking: goal accepted!");
    }
}

void TurtleClient::callback_cameraDocking_result(const rclcpp_action::ClientGoalHandle<action_interfaces::action::Dock>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "[CD] Goal succeeded");
            isCameraDocking = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "[CD] Goal was aborted");
            isCameraDocking = false;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "[CD] Goal was canceled");
            isCameraDocking = false;
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "[CD] Unknown result code");
            isCameraDocking = false;
            return;
    }
}

// ----------------------------------- moveLinear callbacks -------------------------------------------------------------
void TurtleClient::callback_createMoveLinear_goal_response(std::shared_future<rclcpp_action::ClientGoalHandle<DriveDistance>::SharedPtr> future)
{
    auto goal_handle = future.get();
    if(!goal_handle)
    {
        RCLCPP_INFO(get_logger(), "[ML] MoveLinear: goal rejected!");
        isMoveLinear = false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "[ML] MoveLinear: goal accepted!");
    }
}

void TurtleClient::callback_createMoveLinear_result(const rclcpp_action::ClientGoalHandle<DriveDistance>::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "[ML] Goal succeeded");
            isMoveLinear = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "[ML] Goal was aborted");
            isMoveLinear = false;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "[ML] Goal was canceled");
            isMoveLinear = false;
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "[ML] Unknown result code");
            isMoveLinear = false;
            return;
    }
}
