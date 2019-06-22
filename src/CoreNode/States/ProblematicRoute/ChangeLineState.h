#ifndef PROJECT_CHANGELINE_H
#define PROJECT_CHANGELINE_H

#include "../../StateMachine.h"
#include "../../Data.h"

class ChangeLineState : public State<Data> {
public:
    ChangeLineState(int dir, std::chrono::milliseconds duration)
            : _duration(duration), _dir(dir) {}

    std::string name() const override {
        return "Change line";
    }

    void update(Data & data) override {
        if(_firstUpdate) {
            _firstUpdate = false;
            _deathTime = std::chrono::steady_clock::now() + _duration;
        }

        data.lockedSteering= true;
        data.steering = _dir*30;

        if(_deathTime <= std::chrono::steady_clock::now())
            State<Data>::_stateMachine->pop();
    }

private:
    int _dir;
    bool _firstUpdate = true;
    std::chrono::milliseconds _duration;
    std::chrono::steady_clock::time_point _deathTime;

};

#endif //PROJECT_CHANGELINE_H
