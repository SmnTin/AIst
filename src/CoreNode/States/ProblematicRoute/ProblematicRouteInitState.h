//
// Created by smntin on 6/11/19.
//

#ifndef PROJECT_PROBLEMATICROUTEINITSTATE_H
#define PROJECT_PROBLEMATICROUTEINITSTATE_H

#include "../../StateMachine.h"
#include "../../Data.h"

#include "ProblematicRouteState.h"
#include "../Common/RedSignalState.h"

class ProblematicRouteInitState : public State<Data> {
public:
    ProblematicRouteInitState() = default;

    std::string name() const override {
        return "Problematic Route Init";
    }

    void update(Data & data) override {
        State<Data>::_stateMachine->push(std::make_shared<ProblematicRouteState>());
//        State<Data>::_stateMachine->push(std::make_shared<RedSignalState>());
    }
};

#endif //PROJECT_PROBLEMATICROUTEINITSTATE_H
