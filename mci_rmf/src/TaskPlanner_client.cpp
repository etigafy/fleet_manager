#include "TaskPlanner.h"

// #define DEBUG_MSGS_CLIENT
#define LOWER_CHARGING_THRESHOLD 0.15 // % charge
#define UPPER_CHARGING_THRESHOLD 0.20 // % charge
#define IDLE_THRESHOLD 2500 // ms

// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_clientState(const ClientState msg)
{
    // handle client feedback
    if(clientMap.find(msg.id) == clientMap.end())
    {
        // add new client
        Client* client = new Client();
        updateClientData(msg, client);

        // initialize client status and idle state
        client->status = CLIENT_READY;
        client->isOnline = true;
        client->isIdle = false;
        client->isCharging = false;

        // init idle timer
        client->idleTimer = std::chrono::steady_clock::now();

        // add client to clientMap
        clientMap.insert(std::pair<int, Client*>(client->ID, client));
        printf("[CLI] Added %s\n", CustomTypes::getClientString(client->ID));
    }
    else
    {
        // update client
        Client* client = clientMap.find(msg.id)->second;
        updateClientData(msg, client);

        #ifdef DEBUG_MSGS
            printf("Updated client: %ld\n", msg.id); 
        #endif
    }
    return;
}

// ------------------------------------------------------------------------------------------------
void TaskPlanner::updateClientData(const ClientState msg, Client* client)
{
    if( !client->isOnline )
    {
        client->isOnline = true;
        printf("[CLI] %s online.\n", CustomTypes::getClientString(client->ID));
    }

    client->ID = msg.id;
    client->batteryPercent = msg.batterypercent;
    client->isCharging = msg.ischarging;
    client->position.x = msg.pose.pose.position.x;
    client->position.y = msg.pose.pose.position.y;
    client->position.theta = quaternion_2_yaw(msg.pose.pose.orientation);
    client->lastStateUpdate = std::chrono::steady_clock::now();

    // if task is finished, update status
    if( !msg.taskfinished ) return;

    // set status and reset idletimer
    client->status = CLIENT_READY;
    client->idleTimer = std::chrono::steady_clock::now();

    if( !client->isIdle )
    {
        // free up owned landmarks
        for(auto e : client->landmarkOwnershipList)
            e->isOccupiedBy = 0;
        client->landmarkOwnershipList.clear();
    }

    // check if there are tasks in buffer
    if( client->taskBuffer.size() == 0 ) return;

    // erase completed task 
    client->taskBuffer.erase(client->taskBuffer.begin());

    // check if taskBuffer is empty, if so, mark taskGroup as complete
    if( client->taskBuffer.size() == 0 )
    {
        deliveryMap.at(client->activeDeliveryID).at(client->activeTaskGroupID)->isComplete = true;
    }
}

float TaskPlanner::quaternion_2_yaw(geometry_msgs::msg::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_clientTimeout()
{
    using iterator = std::map<int, Client*>::iterator;
    std::chrono::steady_clock::time_point timeoutTimepoint = std::chrono::steady_clock::now() - std::chrono::milliseconds(2500);
    std::vector<iterator> timeoutList;

    // check for timeouts
    for(iterator it = clientMap.begin(); it != clientMap.end(); it++)
    {
        if( (it->second->lastStateUpdate < timeoutTimepoint) && (it->second->isOnline) ) timeoutList.push_back(it);
    }

    // set clients offline
    for(iterator e : timeoutList)
    {
        printf("[CLI] %s offline.\n", CustomTypes::getClientString(e->second->ID));
        e->second->isOnline = false;
        // clientMap.erase(e);
    }

    #ifdef DEBUG_MSGS 
        printf("Remaining clients: %ld\n", clientMap.size());
    #endif
}

// ------------------------------------------------------------------------------------------------
void TaskPlanner::publishMsg_ClientTask(Client* client, TaskType taskType)
{
    // generate task
    auto msg = ClientTask();
    msg.clientid = client->ID;
    msg.tasktype = taskType;

    Landmark* landmark = client->landmarkOwnershipList.front();

    // assign position
    msg.x = landmark->getPosition().position.x;
    msg.y = landmark->getPosition().position.y;
    msg.theta = landmark->getPosition().position.theta;

    msg.pose.pose.position.x = landmark->getPosition().position.x;
    msg.pose.pose.position.y = landmark->getPosition().position.y;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation.w = cos((landmark->getPosition().position.theta / 180 * M_PI) / 2);
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = sin((landmark->getPosition().position.theta / 180 * M_PI) / 2);

    // publish clientTask message
    publisher_clientTask->publish(msg);

    printf("[CLI] Sending %s to %s(%d)\n", CustomTypes::getClientString(client->ID), CustomTypes::getString(landmark->getType()), landmark->getID()); // idle station id
}

// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_clientInterface()
{
    // check up on client status
    std::chrono::steady_clock::time_point idleTimepoint = std::chrono::steady_clock::now() - std::chrono::milliseconds(IDLE_THRESHOLD);
    std::map<int, Client*>::iterator it = clientMap.begin();

    for( ; it != clientMap.end(); it++)
    {
        Client* client = it->second;

        #ifdef DEBUG_MSGS_CLIENT
            printf("%s: %s, idle: %i, buffer: %ld\n", CustomTypes::getClientString(client->ID), CustomTypes::getString(client->status), client->isIdle, client->taskBuffer.size());
        #endif

        // --- remove charging flag if upper threshold is reached and new tasks can be accepted
        if( client->batteryPercent >= UPPER_CHARGING_THRESHOLD )
            client->isCharging = false;

        // --- abort if client is busy
        if( client->status != CLIENT_READY ) continue;

        // --- abort if client is charging
        if( (client->isCharging) && (client->batteryPercent < UPPER_CHARGING_THRESHOLD) ) continue;
        
        // --- prio1: empty taskBuffer
        if( client->taskBuffer.size() > 0 )
        {
            // select next task from activeTaskGroup
            Task* nextTask = client->taskBuffer.front();

            // try to book landmark
            if( bookLandmark(client, nextTask->targetLandmark->getType(), nextTask->targetLandmark->getID()) < 0 ) return;

            // publish task
            publishMsg_ClientTask(client, nextTask->type);

            // set status and idle state
            client->status = CLIENT_BUSY;
            client->isIdle = false;
            
            // printf("[CLI] Sent %s new task: %s\n", CustomTypes::getClientString(client->ID), CustomTypes::getString(nextTask->type));
            continue;
        }

        // --- prio2: charge
        // check battery level and if client is already charging
        if( (client->batteryPercent < LOWER_CHARGING_THRESHOLD) && (!client->isCharging) )
        {
            // book charging station landmark
            if( bookLandmark(client, LANDMARK_CHARGER) < 0 ) return;

            // publish charging task to client
            publishMsg_ClientTask(client, TASK_CHARGE);

            // generate charging task
            client->status = CLIENT_BUSY;
            client->isIdle = true;

            // printf("[CLI] Sent %s to charging station: %d\n", CustomTypes::getClientString(client->ID), 1);
            continue;
        }

        // --- prio3: idle
        // check idle timer
        if( client->idleTimer < idleTimepoint && (!client->isIdle) )
        {
            // try to book idle station landmark
            if( bookLandmark(client, LANDMARK_IDLE) < 0 ) return;

            // publish task
            publishMsg_ClientTask(client, TASK_IDLE);

            // set status and idle state
            client->status = CLIENT_BUSY;
            client->isIdle = true;

            continue;
        }
    }
}

// ------------------------------------------------------------------------------------------------
int TaskPlanner::bookLandmark(Client* client, LandmarkType landmark_type, int landmark_id)
{
    // get specific landmark map and iterator
    std::map<int, Landmark*> specificLandmarkMap = landmarkMap.find(landmark_type)->second;
    std::map<int, Landmark*>::iterator it = specificLandmarkMap.begin();

    // assign temporary nullptr
    Landmark* targetLandmark = nullptr;

    // check landmark availability
    if(landmark_id == -1)
    {
        // book any available landmark, e.g. idle, charger
        for( ; it != specificLandmarkMap.end(); it++)
        {
            // check if targetLandmark is booked -> ID = 0 is reserved for free station
            if( (it->second->isOccupiedBy == 0) || (it->second->isOccupiedBy == client->ID) )
            {
                targetLandmark = it->second;
                break;
            }
        }
    }
    else
    {
        // book landmark with specific id
        if( (it = specificLandmarkMap.find(landmark_id)) == specificLandmarkMap.end() ) return -1;

        // check if targetLandmark is booked -> ID = 0 is reserved for free station
        if( (it->second->isOccupiedBy == 0) || (it->second->isOccupiedBy == client->ID) )
        {
            targetLandmark = it->second;
        }
    }
    
    // if landmark is not available, return
    if( targetLandmark == nullptr )
    {
        printf("[CLI] Landmark %s(%d) not available for client %s!\n", CustomTypes::getString(landmark_type), landmark_id, CustomTypes::getClientString(client->ID));
        return -1;
    }

    // free up owned landmarks
        for(auto e : client->landmarkOwnershipList)
            e->isOccupiedBy = 0;
        client->landmarkOwnershipList.clear();

    // book landmark
    targetLandmark->isOccupiedBy = client->ID;
    client->landmarkOwnershipList.push_back(targetLandmark);

    return 0;
}