#ifndef PROJECT_UNKNOWNPATHINITSTATE_H
#define PROJECT_UNKNOWNPATHINITSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "UnknownPathState.h"
#include "../Common/RedSignalState.h"

class UnknownPathInitState : public State<Data> {
public:
    UnknownPathInitState() = default;

    std::string name() const override {
        return "Unknown Path Init";
    }

    void update(Data & data) override {
        State<Data>::_stateMachine->push(std::make_shared<UnknownPathState>());
        State<Data>::_stateMachine->push(std::make_shared<RedSignalState>());
    }
};

#endif //PROJECT_UNKNOWNPATHINITSTATE_H
