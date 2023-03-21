#pragma once

#include <iostream>

// ------------------------------------------------------------------------------------------------
enum ClientStatus : int
{
    CLIENT_READY,
    CLIENT_BUSY,
    CLIENT_CHARGING
};

enum LandmarkType : int
{
    LANDMARK_NONE,
    LANDMARK_MACHINE_PICKUP,
    LANDMARK_MACHINE_DROPOFF,
    LANDMARK_PICKUP,
    LANDMARK_DROPOFF,
    LANDMARK_CHARGER,
    LANDMARK_IDLE
};

enum PackageType : int
{
    PACKAGE_SINGLE
};

enum TaskType : int
{
    TASK_NONE,
    TASK_IDLE,
    TASK_CHARGE,
    TASK_PICKUP,
    TASK_DROPOFF
};

// ------------------------------------------------------------------------------------------------
struct Position
{
    float x, y, theta;
};

struct location
{
    bool isBusy = false;
    Position position;
};

// ------------------------------------------------------------------------------------------------
class CustomTypes
{
public:
    static const char* getString(ClientStatus enumValue)
    {
        switch (enumValue)
        {
        case CLIENT_READY:
            return "READY";
        case CLIENT_BUSY:
            return "BUSY";
        case CLIENT_CHARGING:
            return "CHARGING";            
        default:
            printf("WARNING - unknown enumValue!\n");
            return nullptr;
        }
    }

    static const char* getString(TaskType enumValue)
    {
        switch (enumValue)
        {
        case TASK_IDLE:
            return "IDLE";
        case TASK_PICKUP:
            return "PICKUP";
        case TASK_DROPOFF:
            return "DROPOFF";     
        case TASK_CHARGE:
            return "CHARGE"; 
        default:
            printf("WARNING - unknown enumValue!\n");
            return nullptr;
        }
    }

    static const char* getString(LandmarkType enumValue)
    {
        switch (enumValue)
        {
        case LANDMARK_IDLE:
            return "IDLE";
        case LANDMARK_MACHINE_PICKUP:
            return "MACHINE_PICKUP";
        case LANDMARK_MACHINE_DROPOFF:
            return "MACHINE_DROPOFF";
        case LANDMARK_PICKUP:
            return "PICKUP";            
        case LANDMARK_DROPOFF:
            return "DROPOFF";
        case LANDMARK_CHARGER:
            return "CHARGER"; 
        default:
            printf("WARNING - unknown ClientStatus!\n");
            return nullptr;
        }
    }

    static const char* getClientString(int value)
    {
        return std::string("[TT" + std::to_string(value) + "]").c_str();
    }
};