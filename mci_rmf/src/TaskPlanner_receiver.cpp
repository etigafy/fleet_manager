#include "TaskPlanner.h"
#include "Package.h"


// ------------------------------------------------------------------------------------------------
void TaskPlanner::callback_deliveryRequest(const std::shared_ptr<Gui::Request> request,
                                        std::shared_ptr<Gui::Response> response)
{
    std::vector<Task*> taskVector;
    Task* task = nullptr;
    std::map<int, Landmark*>::iterator it;

    //// pickup task
    it = landmarkMap.find(LANDMARK_PICKUP)->second.find(request->pickupid);

    // check if ID is valid
    if(it == landmarkMap.find(LANDMARK_PICKUP)->second.end())
    {
        printf("ERROR - unknown pickup ID: %d!\n", request->pickupid);
        response->isaccepted = false;
        return;
    }

    // add pickup task
    task = new Task(TASK_PICKUP);
    task->targetLandmark = it->second;
    taskVector.push_back(task);

    //// machine tasks
    for(int e : request->machineidarray)
    {
        it = landmarkMap.find(LANDMARK_MACHINE_PICKUP)->second.find(e);

        // check if ID is valid
        if(it == landmarkMap.find(LANDMARK_MACHINE_PICKUP)->second.end())
        {
            printf("ERROR - unknown machine ID: %d!\n", e);
            response->isaccepted = false;
            return;
        }

        it = landmarkMap.find(LANDMARK_MACHINE_DROPOFF)->second.find(e);

        // check if ID is valid
        if(it == landmarkMap.find(LANDMARK_MACHINE_DROPOFF)->second.end())
        {
            printf("ERROR - unknown machine ID: %d!\n", e);
            response->isaccepted = false;
            return;
        }

        // drop package at machine
        task = new Task(TASK_DROPOFF);
        task->targetLandmark = landmarkMap.find(LANDMARK_MACHINE_DROPOFF)->second.find(e)->second;
        taskVector.push_back(task);

        // pick package from machine
        task = new Task(TASK_PICKUP);
        task->targetLandmark = landmarkMap.find(LANDMARK_MACHINE_PICKUP)->second.find(e)->second;
        taskVector.push_back(task);
    }

    //// dropoff task
    it = landmarkMap.find(LANDMARK_DROPOFF)->second.find(request->dropoffid);

    // check if ID is valid
    if(it == landmarkMap.find(LANDMARK_DROPOFF)->second.end())
    {
        printf("ERROR - unknown dropoff ID: %d!\n", request->dropoffid);
        response->isaccepted = false;
        return;
    }

    // add dropoff task
    task = new Task(TASK_DROPOFF);
    task->targetLandmark = landmarkMap.find(LANDMARK_DROPOFF)->second.find(request->dropoffid)->second;
    taskVector.push_back(task);


    //// format local tasks
    std::map<int, TaskGroup*> localTaskGroupMap;

    // create task groups
    for(size_t i = 0; i < size_t(taskVector.size() / 2); i++)
    {
        TaskGroup* tmpTaskGroup = new TaskGroup;
        tmpTaskGroup->groupID = i + 1;
        tmpTaskGroup->isSerialized = true;
        tmpTaskGroup->taskList.push_back(taskVector[2*i]);
        tmpTaskGroup->taskList.push_back(taskVector[2*i+1]);

        // assign processing time
        if(tmpTaskGroup->taskList[0]->targetLandmark->getType() == LANDMARK_MACHINE_PICKUP)
        {
            tmpTaskGroup->isProcessing = false;
            const unsigned long p_time = static_cast<unsigned long>(tmpTaskGroup->taskList[0]->targetLandmark->getProcessingTime());
            tmpTaskGroup->processingTime = std::chrono::milliseconds(p_time);
        }

        localTaskGroupMap.insert(std::pair<int, TaskGroup*>(tmpTaskGroup->groupID, tmpTaskGroup));
    }

    // add delivery to deliveryVector
    deliveryMap.insert(std::pair<int, std::map<int, TaskGroup*>>(deliveryCounter, localTaskGroupMap));
    deliveryCounter++;

    // update response
    response->isaccepted = true;

    printf("[RCV] Received new delivery, %ld group(s), %ld task(s)!\n", localTaskGroupMap.size(), taskVector.size());
}