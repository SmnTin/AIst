#ifndef PROJECT_QUALIFICATIONTURNINGSTATE_H
#define PROJECT_QUALIFICATIONTURNINGSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

class QualificationTurningState : public State<Data> {
public:
    explicit QualificationTurningState(int8_t direction = 0,
            std::chrono::milliseconds duration = std::chrono::milliseconds{0})
            : _duration(duration), _direction(direction) {}

    std::string name() const override {
        return "Qualification Turning";
    }

    void update(Data & data) override {
        if(_firstUpdate) {
            _firstUpdate = false;
            _deathTime = std::chrono::steady_clock::now() + _duration;
        }
        data.direction = _direction;
        data.lockedSteering = true;
        data.steering = _direction*90;
        if(_deathTime <= std::chrono::steady_clock::now())
            State<Data>::_stateMachine->pop();
    }

private:
    int8_t _direction;
    std::chrono::milliseconds _duration;
    std::chrono::steady_clock::time_point _deathTime;
    bool _firstUpdate = true;

};

#endif //PROJECT_QUALIFICATIONTURNINGSTATE_H
