#ifndef PROJECT_CURVYLINE_H
#define PROJECT_CURVYLINE_H


#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "../Common/TurningState.h"

class CurvyLineState : public State<Data> {
public:
    CurvyLineState() = default;

    std::string name() const override {
        return "Problematic Route";
    }

    void update(Data &data) override {
        data.direction = 0;
        data.lockedSteering = true;
        data.steering = 0;
        auto now = std::chrono::steady_clock::now();
        if(data.lostLine)
            _lastLostLineTime = now;
        if(now - _lastLostLineTime > _stabilizationTime)
            State<Data>::_stateMachine->pop();
    }

private:
    const std::chrono::milliseconds _stabilizationTime{1000};
    std::chrono::steady_clock::time_point _lastLostLineTime;
};


#endif //PROJECT_CURVYLINE_H
