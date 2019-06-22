#ifndef PROJECT_UNKNOWNPATHSTATE_H
#define PROJECT_UNKNOWNPATHSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "../Common/TurningState.h"

class UnknownPathState : public State<Data> {
public:
    UnknownPathState() = default;

    std::string name() const override {
        return "Unknown Path";
    }

    void update(Data &data) override {
        data.power = _defaultSpeed;
        data.direction = 0;

        _reactToLights(data);
        _reactToSigns(data);
    }

private:
    const std::chrono::milliseconds _slowDownTime{2000};
    const std::chrono::milliseconds _turningTime{4000};

    float _defaultSpeed = 0.5f;

    bool _turnRightSeen = false;
    bool _turnLeftSeen = false;

    void _reactToLights(Data &data) {
        reactToLights(data, State<Data>::_stateMachine);
    }

    void _reactToSigns(Data &data) {
        for (auto &sign : data.signs) {
            if (sign.type == aist::TrafficSign::TYPE_RIGHT
                && sign.distance < 0.5) {

                _turnRightSeen = true;
            }

            if (sign.type == aist::TrafficSign::TYPE_LEFT
                && sign.distance < 0.5) {

                _turnLeftSeen = true;
            }
        }
        if(data.forkLine) {
            if(_turnRightSeen || _turnLeftSeen) {
                if(_turnRightSeen)
                    State<Data>::_stateMachine->push(std::make_shared<TurningState>(1, _turningTime));
                else
                    State<Data>::_stateMachine->push(std::make_shared<TurningState>(-1, _turningTime));
                _turnLeftSeen = _turnRightSeen = false;
            }
        }
    }
};

#endif //PROJECT_UNKNOWNPATHSTATE_H
