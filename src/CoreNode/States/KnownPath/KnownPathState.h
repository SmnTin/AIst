#ifndef PROJECT_KNOWNPATHSTATE_H
#define PROJECT_KNOWNPATHSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "../Common/reactToSlowdown.h"
#include "../Common/TurningState.h"

class KnownPathState : public State<Data> {
public:
    KnownPathState() {
        _forkLastSeen = std::chrono::steady_clock::now() - _forkDelay;

        pushPath({-1, 0, -1, 1});
    }

    std::string name() const override {
        return "Known Path";
    }

    void pushPath(const std::vector<int> & path) {
        for(const int & t : path)
            _path.push(t);
    }

    void update(Data &data) override {
        data.power = _defaultSpeed;
        data.direction = 0;
        data.lockedSteering = false;
        data.returnToLineAlgo = true;

        _reactToLights(data);
        _reactToSigns(data);
    }

private:
    const std::chrono::milliseconds _slowDownTime{2000};
    const std::chrono::milliseconds _ignoranceTime{1000};
    const std::chrono::milliseconds _turningTime{2000};

    const std::chrono::milliseconds _forkDelay{3000};
    std::chrono::steady_clock::time_point _forkLastSeen;

    std::queue<int> _path;

    float _defaultSpeed = 0.4f;

    void _reactToLights(Data &data) {
        bool redLights = false;
        while (!data.trafficLights.empty()) {
            if ((data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_RED ||
                 data.trafficLights.front().type == aist::TrafficLightsInfo::TYPE_AMBER)
                && data.trafficLights.front().distance < 0.5)

                redLights = true;

            data.trafficLights.pop();
        }

        if (redLights) {
//            State<Data>::_stateMachine->push(std::make_shared<SlowDownState>(_defaultSpeed, _forkDelay));
            State<Data>::_stateMachine->push(std::make_shared<RedSignalState>());
        }
    }

    void _reactToSigns(Data &data) {
        reactToSlowdownSigns(data, State<Data>::_stateMachine, _defaultSpeed, _slowDownTime);

        auto now = std::chrono::steady_clock::now();
        if(data.forkLine && now - _forkLastSeen > _forkDelay) {
            _forkLastSeen = now;

            int dir = 0;
            if(!_path.empty()) {
                dir = _path.front();
                _path.pop();
            }

            if(dir > 0)
                State<Data>::_stateMachine->push(std::make_shared<TurningState>(1, _turningTime));
            else if(dir < 0)
                State<Data>::_stateMachine->push(std::make_shared<TurningState>(-1, _turningTime));
        }
    }
};

#endif //PROJECT_KNOWNPATHSTATE_H
