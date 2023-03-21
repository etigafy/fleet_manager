#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "Landmark.h"


// ------------------------------------------------------------------------------------------------
class LandmarkFileStream
{
public:
    LandmarkFileStream(std::string filePath, LandmarkType landmarkType)
    {
        this->filePath = filePath;
        this->landmarkType = landmarkType;
    }

    int readFile(std::map<int, Landmark*> *specificLandmarkMap);

private:
    std::string filePath;
    LandmarkType landmarkType = LANDMARK_NONE;
    std::vector<std::string> fileContent;
    size_t read_pointer = 0;
    size_t num_landmarks;

    int readInt();
    float readFloat();
};

// ------------------------------------------------------------------------------------------------
int LandmarkFileStream::readFile(std::map<int, Landmark*> *specificLandmarkMap)
{
    std::ifstream file;

    try
    {
        file.open(filePath);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    // extract lines
    for(std::string line; getline(file, line);)
    {
        fileContent.push_back(line);
    }

    if(fileContent.size() == 0)
    {
        printf("ERROR - no landmarkData, missing file %d?\n", landmarkType);
        return -1;
    }
    // remove empty and commented lines
    std::vector<std::string>::iterator it = fileContent.begin();
    for( ; it != fileContent.end(); )
    {
        char firstChar = (*it)[0];
        if( (firstChar == '#') || ((*it) == "") ) fileContent.erase(it);
        else it++;
    }

    // extract numerical values
    for(size_t i = 0; i < fileContent.size(); i++)
    {
        std::string dilimeter = "=";
        size_t start = fileContent[i].find(dilimeter) + 1;
        size_t end = fileContent[i].length() - 1;
        fileContent[i] = fileContent[i].substr(start, end);
    }

    // cleanup
    file.close();

    // write landmarks to map
    num_landmarks = readInt();
    // printf("num_landmarks: %ld\n", num_landmarks);

    for(size_t i = 0; i < num_landmarks; i++)
    {
        Landmark *landmark = new Landmark(landmarkType);
        location tempLocation;

        // landmark ID
        landmark->setID(readInt());

        // tempLocation
        tempLocation.position.x = readFloat();
        tempLocation.position.y = readFloat();
        tempLocation.position.theta = readFloat();
        landmark->setPosition(tempLocation);

        // processing time
        if((landmarkType == LANDMARK_MACHINE_PICKUP) || (landmarkType == LANDMARK_MACHINE_DROPOFF))
        {
            landmark->setProcessingTime(readInt());
        }

        // printf("id: %d\n", landmark->getID());

        specificLandmarkMap->insert(std::pair<int, Landmark*>(landmark->getID(), landmark));
    }

    if( num_landmarks != specificLandmarkMap->size() )
    {
        printf("ERROR - landmark_num mismatch (%ld/%ld) in landmarkType: %d, duplicate IDs?\n", specificLandmarkMap->size(), num_landmarks, landmarkType);
        return -1;
    }

    return 0;
}

// ------------------------------------------------------------------------------------------------
int LandmarkFileStream::readInt()
{ 
    int value = atoi(fileContent[read_pointer].c_str());
    read_pointer++;
    return value;
}
float LandmarkFileStream::readFloat()
{
    float value = atof(fileContent[read_pointer].c_str());
    read_pointer++;
    return value;
}