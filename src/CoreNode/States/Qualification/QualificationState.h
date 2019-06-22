#ifndef PROJECT_QUALIFICATIONSTATE_H
#define PROJECT_QUALIFICATIONSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "TurningState.h"

class QualificationState : public State<Data> {
public:
    QualificationState() = default;

    std::string name() const override {
        return "Qualification";
    }

    void update(Data & data) override {
        data.power = _defaultSpeed;
        data.direction = 0;
        data.lockedSteering = false;
        data.returnToLineAlgo = false;

        _reactToLights(data);
        _reactToSigns(data);
    }

private:
    const std::chrono::milliseconds _slowDownTime{2000};
    const std::chrono::milliseconds _turningTime{2000};
    const std::chrono::milliseconds _turningTime2{1000};

    float _defaultSpeed = 0.5f;

    void _reactToLights(Data & data) {
        reactToLights(data, State<Data>::_stateMachine);
    }

    void _reactToSigns(Data & data) {
        reactToSlowdownSigns(data, State<Data>::_stateMachine, _defaultSpeed, _slowDownTime);
        for (auto &sign : data.signs) {
            if (sign.type == aist::TrafficSign::TYPE_RIGHT
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _slowDownTime));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(1, _turningTime));
            }

            if (sign.type == aist::TrafficSign::TYPE_LEFT
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _slowDownTime));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(-1, _turningTime));
            }

            if (sign.type == aist::TrafficSign::TYPE_CURVY
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _slowDownTime));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(0, _turningTime2));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(1, _turningTime2));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(-1, _turningTime2));
            }

            if (sign.type == aist::TrafficSign::TYPE_PASS_LEFT
                && sign.distance < 0.5) {

                State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _slowDownTime));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(0, _turningTime2));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(1, _turningTime2));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(0, _turningTime2));
                State<Data>::_stateMachine->push(std::make_shared<QualificationTurningState>(-1, _turningTime2));
            }
        }
    }

};

#endif //PROJECT_QUALIFICATIONSTATE_H
