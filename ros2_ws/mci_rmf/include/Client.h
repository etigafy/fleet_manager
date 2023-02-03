#pragma once

#include <chrono>
#include <vector>

#include "Landmark.h"
#include "Task.h"


// ------------------------------------------------------------------------------------------------
class Client
{
public:
    int ID;
    ClientStatus status;
    bool isOnline = true;
    bool isIdle = false;
    bool isCharging = false;
    float batteryPercent;
    Position position;
    std::chrono::steady_clock::time_point lastStateUpdate;
    std::chrono::steady_clock::time_point idleTimer;
    int activeDeliveryID = -1;
    int activeTaskGroupID = -1;
    std::vector<Task*> taskBuffer;
    std::vector<Landmark*> landmarkOwnershipList;
};