#ifndef PROJECT_CURVYLINE_H
#define PROJECT_CURVYLINE_H


#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "../Common/TurningState.h"

class CurvyLineState : public State<Data> {
public:
    CurvyLineState() {
        _start = std::chrono::steady_clock::now();
    }

    std::string name() const override {
        return "Problematic Route";
    }

    const float _defaultSpeed = 0.6f;

    void update(Data &data) override {
        data.direction = 1;
//        data.useDeviation2 = true;

        data.lockedSteering = true;

//        data.pid.kp = 0.1;
//        data.pid.ki = 0;
//        data.pid.kd = 0;

        data.power = _defaultSpeed;
        auto now = std::chrono::steady_clock::now();

        if(now - _start < _correctionTime)
            data.steering = -3;
        else
            data.steering = 2;

        if(data.lostLine)
            _lastLostLineTime = now;
        if(now - _lastLostLineTime > _stabilizationTime)
            State<Data>::_stateMachine->pop();
    }

private:
    const std::chrono::milliseconds _stabilizationTime{1000};
    const std::chrono::milliseconds _correctionTime{1000};
    std::chrono::steady_clock::time_point _lastLostLineTime;
    std::chrono::steady_clock::time_point _start;
};


#endif //PROJECT_CURVYLINE_H
