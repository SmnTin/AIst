#ifndef PROJECT_QUALIFICATIONINITSTATE_H
#define PROJECT_QUALIFICATIONINITSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "QualificationState.h"
#include "../Common/RedSignalState.h"

class QualificationInitState : public State<Data> {
public:
    QualificationInitState() = default;

    std::string name() const override {
        return "Qualification Init";
    }

    void update(Data & data) override {
        State<Data>::_stateMachine->push(std::make_shared<QualificationState>());
//        State<Data>::_stateMachine->push(std::make_shared<RedSignalState>());
    }
};

#endif //PROJECT_QUALIFICATIONINITSTATE_H
