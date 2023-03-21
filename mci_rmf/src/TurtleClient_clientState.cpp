#include "TurtleClient.h"

void TurtleClient::callback_clientState()
{
    auto msg = ClientState();
    msg.id = clientID;
    msg.batterypercent = batteryPercent;
    msg.pose = clientPose;

    if( isDocked != -1 ) msg.ischarging = static_cast<bool>(isDocked);
    else msg.ischarging = false;

    // report finished task
    msg.taskfinished = taskFinished;
    taskFinished = false;

    // publish feedback
    publisher_clientState->publish(msg);
    // printf("state: %d\n", counter);
    counter++;
}

void TurtleClient::callback_dock(Dock msg)
{
    msg.is_docked ? isDocked = 1 : isDocked = 0;
    // printf("dock, %i\n", *isDocked);
}

void TurtleClient::callback_batteryState(BatteryState msg)
{
    batteryPercent = msg.percentage;
    RCLCPP_INFO(get_logger(), "[Battery] %.1f%% %s", batteryPercent * 100, isDocked == 1 ? "-> charging" : "");
}

void TurtleClient::callback_odom(Odometry msg)
{
    clientPose.header = msg.header;
    clientPose.pose = msg.pose.pose;
    // printf("[odom]\n");
}