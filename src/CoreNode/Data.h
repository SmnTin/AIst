#ifndef PROJECT_DATA_H
#define PROJECT_DATA_H

#include <ros/ros.h>
#include <aist/LineInfo.h>
#include <aist/Motors.h>
#include <aist/LineControl.h>
#include <aist/TrafficSignsInfo.h>
#include <aist/TrafficLightsInfo.h>

#include "PID.h"

#include <queue>
#include <vector>

struct Data {
    bool lockedSteering = false;
    float steering = 0;
    float power = 0;
    int8_t direction = 0;

    bool lostLine = false;
    bool stopLine = false;
    bool forkLine = false;
    bool returnToLineAlgo = true;
    bool useDeviation2 = false;

    bool dummy = false;

    std::queue<aist::TrafficSign> appearedSigns, disappearedSigns;
    std::vector<aist::TrafficSign> signs;

    std::queue<aist::TrafficLightsInfo> trafficLights;

    double PID_P = 1;
    double PID_I = 1;
    double PID_D = 1;
    double PID_B = 1;
    double PID_K = 1;
    PID pid;
};

#endif //PROJECT_DATA_H
