#ifndef PROJECT_SLOWDOWNSTATE_H
#define PROJECT_SLOWDOWNSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

class SlowDownState : public State<Data> {
public:
    SlowDownState(float power, std::chrono::milliseconds duration, bool lockedSteering = false)
            : _power(power), _duration(duration), _lockedSteering(lockedSteering) {}

    std::string name() const override {
        return "Slowdown";
    }

    void update(Data & data) override {
        if(_firstUpdate) {
            _firstUpdate = false;
            _deathTime = std::chrono::steady_clock::now() + _duration;
        }
        data.lockedSteering = _lockedSteering;
        data.power = _power;
        if(_deathTime <= std::chrono::steady_clock::now())
            State<Data>::_stateMachine->pop();
    }

private:
    float _power;
    bool _lockedSteering;
    bool _firstUpdate = true;
    std::chrono::milliseconds _duration;
    std::chrono::steady_clock::time_point _deathTime;

};

#endif //PROJECT_SLOWDOWNSTATE_H
