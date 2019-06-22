#ifndef PROJECT_REDSIGNAL_H
#define PROJECT_REDSIGNAL_H

#include "../../StateMachine.h"
#include "../../Data.h"

class RedSignalState : public State<Data> {
public:
    RedSignalState() = default;

    std::string name() const override {
        return "Red Signal";
    }

    void update(Data & data) override {
        data.power = 0;
        bool greenLights = false, redLights = false;

        while(!data.trafficLights.empty()) {
            if(data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_RED)
                redLights = true;
            if(data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_GREEN)
                greenLights = true;

            data.trafficLights.pop();
        }

        if(greenLights && !redLights)
            State<Data>::_stateMachine->pop();
    }

};


#endif //PROJECT_REDSIGNAL_H
