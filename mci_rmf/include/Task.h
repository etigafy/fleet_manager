#pragma once

#include <vector>

#include <stdio.h>
#include <CustomTypes.h>
#include <Landmark.h>

// ------------------------------------------------------------------------------------------------
class Task
{
public:
    Task(TaskType in_type) : type(in_type) {};

    TaskType type = TASK_NONE;
    Landmark* targetLandmark = nullptr;
    bool isComplete = false;
    int isHandledBy = 0;
};

// ------------------------------------------------------------------------------------------------
class TaskGroup
{
public:
    std::vector<Task*> taskList;
    int groupID = -1;
    bool isSerialized = false;
    bool isComplete = false;
    int isHandledBy = 0;

    std::chrono::milliseconds processingTime = std::chrono::milliseconds(0); // ms
    bool isProcessing = true;
    std::chrono::steady_clock::time_point processingStartTime;
};

// ------------------------------------------------------------------------------------------------
class Delivery
{
public:
    std::vector<TaskGroup*> taskGroups;
    int deliveryID = -1;
    bool isSerialized = false;
};