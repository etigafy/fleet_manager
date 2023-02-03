#include "TaskPlanner.h"
#include "LandmarkFileStream.h"

#define machinePickupFilePath   "/home/h/taskPlannerData/machinePickup.dat"
#define machineDropoffFilePath  "/home/h/taskPlannerData/machineDropoff.dat"
#define pickupStationFilePath   "/home/h/taskPlannerData/pickupStations.dat"
#define dropoffStationFilePath  "/home/h/taskPlannerData/dropoffStations.dat"
#define chargerFilePath         "/home/h/taskPlannerData/chargers.dat"
#define idleStationFilePath     "/home/h/taskPlannerData/idleStations.dat"

// ------------------------------------------------------------------------------------------------
int TaskPlanner::loadLandmarks()
{
    const size_t num_landmarkTypes = 6;
    std::string filePathArray[num_landmarkTypes] = { machinePickupFilePath, machineDropoffFilePath, pickupStationFilePath,
                                                     dropoffStationFilePath, chargerFilePath, idleStationFilePath };
    LandmarkType landmarkTypeArray[num_landmarkTypes] = { LANDMARK_MACHINE_PICKUP, LANDMARK_MACHINE_DROPOFF, LANDMARK_PICKUP,
                                                          LANDMARK_DROPOFF, LANDMARK_CHARGER, LANDMARK_IDLE };

    std::map<int, Landmark*> tempMap;

    // load Landmarks
    for(size_t i = 0; i < num_landmarkTypes; i++)
    {
        tempMap.clear();
        LandmarkFileStream fileStream(filePathArray[i], landmarkTypeArray[i]);
        if(fileStream.readFile(&tempMap) < 0) continue;
        landmarkMap.insert(std::pair<LandmarkType, std::map<int, Landmark*>>(landmarkTypeArray[i], tempMap));

        // for(std::map<int, Landmark*>::iterator it = tempMap.begin(); it != tempMap.end(); it++)
        // {
        //     printf("%s, %d\n", CustomTypes::getString(it->second->getType()), it->second->getID());
        // }
    }

    printf("%ld/%ld landmark files read.\n", landmarkMap.size(), num_landmarkTypes);

    if(landmarkMap.size() != num_landmarkTypes) return -1;

    return 0;
}