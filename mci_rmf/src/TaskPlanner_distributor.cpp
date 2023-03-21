#include "TaskPlanner.h"

// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_taskDistributor()
{   
    // printf("[DTS] %ld delivery(s)!\n", deliveryMap.size());
    // for(auto e : deliveryMap)
    // {
    //     printf("[DTS] Delivery(%d): %ld TaskGroups!\n", e.first, e.second.size());
    // }

    // check for outstanding tasks, else return
    if( deliveryMap.size() == 0 ) return;

    // vector of available clients
    std::vector<Client*> availableClientVector;

    // collect all clients with CLIENT_READY status
    for(std::map<int, Client*>::iterator it = clientMap.begin(); it != clientMap.end(); it++)
    {
        Client* client = it->second;

        if( client->status == CLIENT_READY ) availableClientVector.push_back(client);
    }

    // if no clients are available, return
    // printf("Tasks: %ld, Available Clients: %ld\n", taskVector.size(), availableClientVector.size());
    if( availableClientVector.size() == 0 ) return;

    // pick out task to be distributed
    int deliveryID, taskGroupID;
    deliveryID = taskGroupID = -1;
    if(chooseNewTaskGroup(&deliveryID, &taskGroupID) < 0) return;

    TaskGroup* taskGroup = deliveryMap.at(deliveryID).at(taskGroupID);

    // decide which client receives the task -------------------------------------------------------- TODO
    Client* targetClient = availableClientVector.front();

    // assign client to taskGroup
    printf("[DST] Pushing TaskGroup(%d,%d) to %s.\n", deliveryID, taskGroup->groupID, CustomTypes::getClientString(targetClient->ID));
    taskGroup->isHandledBy = targetClient->ID;
    taskGroup->isComplete = false;
    targetClient->activeDeliveryID = deliveryID;
    targetClient->activeTaskGroupID = taskGroupID;

    // push tasks of group to taskBuffer
    for(Task* e : taskGroup->taskList)
        targetClient->taskBuffer.push_back(e);

    // remove task from tasklist
    // taskVector.erase(taskVector.begin());
}

// ------------------------------------------------------------------------------------------------
int TaskPlanner::chooseNewTaskGroup(int* deliveryID, int* taskGroupID)
{
    std::map<int, std::map<int, TaskGroup*>>::iterator d_it = deliveryMap.begin();
    // loop through deliveries
    for( ; d_it != deliveryMap.end(); d_it++)
    {
        std::map<int, TaskGroup*>::iterator t_it = d_it->second.begin();
        // loop through taskGroups
        for( ; t_it != d_it->second.end(); t_it++)
        {
            if( t_it->second->isComplete ) continue;
            if( t_it->second->isHandledBy != 0 ) break;

            // check if processing is required
            if( !t_it->second->isProcessing )
            {
                // start processing the package
                t_it->second->isProcessing = true;
                // start processing timer
                t_it->second->processingStartTime = std::chrono::steady_clock::now();

                printf("[DST] Processing (%d,%d) started -> %ld ms\n", d_it->first, t_it->first, t_it->second->processingTime.count());
                break;
            }


            std::chrono::steady_clock::time_point pTimepoint = std::chrono::steady_clock::now() - t_it->second->processingTime;
            if( pTimepoint < t_it->second->processingStartTime) break;

            // return current taskGroup
            *deliveryID = d_it->first;
            *taskGroupID = t_it->first;
            return 0;
        }
    }

    return -1;
}