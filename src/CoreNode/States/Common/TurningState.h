#ifndef PROJECT_TURNING_H
#define PROJECT_TURNING_H

#include "../../StateMachine.h"
#include "../../Data.h"

class TurningState : public State<Data> {
public:
    TurningState(int8_t direction, std::chrono::milliseconds duration) {
        _direction = direction;
        _deathTime = std::chrono::steady_clock::now() + duration;
    }

    std::string name() const override {
        return "Turning";
    }

    void update(Data & data) override {
        data.direction = _direction;
        if(_deathTime <= std::chrono::steady_clock::now())
            State<Data>::_stateMachine->pop();
    }

private:
    int8_t _direction;
    std::chrono::steady_clock::time_point _deathTime;

};

#endif //PROJECT_TURNING_H
