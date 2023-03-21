#pragma once

#include <stdio.h>


// ------------------------------------------------------------------------------------------------
class Landmark
{
public:
    Landmark(LandmarkType in_type) : type{in_type}{};


    int isOccupiedBy = 0;

    void setID(int id) {this->id = id;};
    int getID() const {return id;};

    LandmarkType getType() const { return type; };

    void setPosition(location pos) {this->position = pos;};

    location getPosition() const {return position;};

    void setProcessingTime(float processingTime) {this->processingTime = processingTime;};
    float getProcessingTime() const {return processingTime;};

protected:
    int id;
    LandmarkType type = LANDMARK_NONE;
    location position;
    float processingTime;
};

// ------------------------------------------------------------------------------------------------