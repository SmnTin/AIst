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
        data.useDeviation2 = false;

        data.pid.kp = data.PID_P;
        data.pid.ki = data.PID_I;
        data.pid.kd = data.PID_D;

        _reactToLights(data);
        _reactToSigns(data);
    }

private:
    const std::chrono::milliseconds _turningTime{1000};
    const std::chrono::milliseconds _drivingAlongTime{2000};
    const std::chrono::milliseconds _changingLineTime{1000};
    const std::chrono::milliseconds _changingLineBackTime{1000};

    float _defaultSpeed = 0.4f;

    void _reactToLights(Data &data) {
//        reactToLights(data, State<Data>::_stateMachine);
    }

    void _reactToSigns(Data &data) {
        bool changeLine = false;
        bool curvyLine = false;

        for (auto &sign : data.signs) {
            if (sign.type == aist::TrafficSign::TYPE_CURVY
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<CurvyLineState>());
                _defaultSpeed = 0.5f;
            }

            if (sign.type == aist::TrafficSign::TYPE_PASS_LEFT
                && sign.distance < 0.5) {

                changeLine = true;
            }
        }
        if(curvyLine) {
        }
        if(changeLine) {
            State<Data>::_stateMachine->push(std::make_shared<ChangeLineState>(1, _changingLineBackTime));
            State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _drivingAlongTime));
            State<Data>::_stateMachine->push(std::make_shared<ChangeLineState>(-1, _changingLineTime));
        }
        if(data.forkLine) {
            State<Data>::_stateMachine->push(std::make_shared<TurningState>(1, _turningTime));
        }
    }
};

#endif //PROJECT_PROBLEMATICROUTESTATE_H
