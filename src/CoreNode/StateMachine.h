#ifndef LINE_NODE_STATEMACHINE_H
#define LINE_NODE_STATEMACHINE_H

#include <chrono>
#include <functional>
#include <stack>
#include <mutex>
#include <thread>
#include <tuple>
#include <queue>
#include <memory>

template<class T>
class StateMachine;

template<class T>
class State {
public:
    virtual void update(T & data) = 0;
    virtual ~State() {};
    virtual std::string name() const {
        return "";
    };
    void setStateMachine(StateMachine<T> * stateMachine) {
        _stateMachine = stateMachine;
    }

protected:
    StateMachine<T> * _stateMachine;
};

template<class T>
class StateMachine {
public:
    void update(T & data) {
        _toClear.clear();

        if (!_states.empty())
            _states.top()->update(data);
    }

    void pop() {
        if(_states.empty())
            throw std::runtime_error("Can't pop empty stack.");

        _toClear.push_back(_states.top());
        _states.pop();
        std::cout << "[State machine] Popped state.\n";
    }

    void push(const std::shared_ptr<State<T>> & state) {
        state->setStateMachine(this);
        _states.push(state);
        std::cout << "[State machine] Pushed \"" << state->name() << "\" state." << "\n";
    }

private:
    std::stack<std::shared_ptr<State<T>>> _states;
    std::vector<std::shared_ptr<State<T>>> _toClear;
};

template<class T, class Q>
class ValueKeeper : public State<T>{
public:
    ValueKeeper(Q & var, Q value)
            : _var(var), _value(value), _runForever(true) {
        _deathTime = std::chrono::steady_clock::now();
    }
    ValueKeeper(Q & var, Q value, std::chrono::milliseconds duration)
            : _var(var), _value(value) {
        _deathTime = std::chrono::steady_clock::now() + duration;
    }
    void update(T & data) override {
        _var = _value;
        if(!_runForever && _deathTime <= std::chrono::steady_clock::now())
            State<T>::_stateMachine->pop();
    }

private:
    Q & _var, _value;
    std::chrono::steady_clock::time_point _deathTime;
    bool _runForever = false;
};

#endif //LINE_NODE_STATEMACHINE_H
