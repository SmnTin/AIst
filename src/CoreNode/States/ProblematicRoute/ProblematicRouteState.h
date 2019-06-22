#ifndef PROJECT_PROBLEMATICROUTESTATE_H
#define PROJECT_PROBLEMATICROUTESTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "../Common/TurningState.h"
#include "CurvyLineState.h"
#include "ChangeLineState.h"

class ProblematicRouteState : public State<Data> {
public:
    ProblematicRouteState() = default;

    std::string name() const override {
        return "Problematic Route";
    }

    void update(Data &data) override {
        data.power = _defaultSpeed;
        data.direction = 0;
        data.lockedSteering = false;

        _reactToLights(data);
        _reactToSigns(data);
    }

private:
    const std::chrono::milliseconds _turningTime{1000};
    const std::chrono::milliseconds _drivingAlongTime{3000};
    const std::chrono::milliseconds _changingLineTime{1000};
    const std::chrono::milliseconds _changingLineBackTime{1500};

    float _defaultSpeed = 0.5f;

    void _reactToLights(Data &data) {
        reactToLights(data, State<Data>::_stateMachine);
    }

    void _reactToSigns(Data &data) {

        for (auto &sign : data.signs) {
            if (sign.type == aist::TrafficSign::TYPE_CURVY
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<CurvyLineState>());
            }

            if (sign.type == aist::TrafficSign::TYPE_PASS_LEFT
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<ChangeLineState>(1, _changingLineBackTime));
                State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _drivingAlongTime));
                State<Data>::_stateMachine->push(std::make_shared<ChangeLineState>(-1, _changingLineTime));
            }
        }
        if(data.forkLine) {
            State<Data>::_stateMachine->push(std::make_shared<TurningState>(1, _turningTime));
        }
    }
};

#endif //PROJECT_PROBLEMATICROUTESTATE_H
