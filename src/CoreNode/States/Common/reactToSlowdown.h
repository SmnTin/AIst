#ifndef PROJECT_REACTTOSLOWDOWNSIGNS_H
#define PROJECT_REACTTOSLOWDOWNSIGNS_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "SlowDownState.h"
#include "RedSignalState.h"

void reactToSlowdownSigns(Data & data, StateMachine<Data> * stateMachine, double defaultSpeed, std::chrono::milliseconds slowDownTime) {
    bool slowdown = false;
    bool enableIgnorance = false;
    double slowdownSpeed = 0;
    for (auto &sign : data.signs) {
        if (sign.type == aist::TrafficSign::TYPE_PED
            && sign.distance < 0.5) {

            slowdown = true;
            slowdownSpeed = 0.2f;

        }

        if (sign.type == aist::TrafficSign::TYPE_STOP
            && sign.distance < 0.35) {

            slowdown = true;
            enableIgnorance = true;
            slowdownSpeed = 0.0f;
        }
    }

    if (slowdown) {
        if(enableIgnorance)
            stateMachine->push(std::make_shared<SlowDownState>(defaultSpeed, slowDownTime));
        stateMachine->push(std::make_shared<SlowDownState>(slowdownSpeed, slowDownTime));
    }
}

void reactToLights(Data & data, StateMachine<Data> * stateMachine) {
    bool redLights = false;
    while (!data.trafficLights.empty()) {
        if ((data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_RED ||
             data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_AMBER)
            && data.trafficLights.front().distance < 0.5)

            redLights = true;

        data.trafficLights.pop();
    }

    if (redLights)
        stateMachine->push(std::make_shared<RedSignalState>());
}

#endif //PROJECT_REACTTOSLOWDOWNSIGNS_H
