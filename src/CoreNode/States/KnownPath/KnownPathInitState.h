#ifndef PROJECT_KNOWNPATHINITSTATE_H
#define PROJECT_KNOWNPATHINITSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "KnownPathState.h"
#include "../Common/RedSignalState.h"

class KnownPathInitState : public State<Data> {
public:
    KnownPathInitState() = default;

    std::string name() const override {
        return "Known Path Init";
    }

    void update(Data & data) override {
        State<Data>::_stateMachine->push(std::make_shared<KnownPathState>());
        State<Data>::_stateMachine->push(std::make_shared<RedSignalState>());
    }
};

#endif //PROJECT_KNOWNPATHINITSTATE_H
