#include "Core.h"

#include "States/Qualification/QualificationInitState.h"
#include "States/UnknownPath/UnknownPathInitState.h"
#include "States/KnownPath/KnownPathInitState.h"
#include "States/ProblematicRoute/ProblematicRouteInitState.h"

Core::Core(ros::NodeHandle &node) : _node(node) {
    node.param("line_info_topic", _lineInfoTopic, (std::string)"/line_info");
    node.param("line_control_topic", _lineControlTopic, (std::string)"/line_control");
    node.param("motors_info_topic", _motorsInfoTopic, (std::string)"/motors_info");
    node.param("traffic_signs_info_topic", _trafficSignsInfoTopic, (std::string)"/traffic_signs_info");
    node.param("traffic_lights_info_topic", _trafficLightsInfoTopic, (std::string)"/traffic_lights_info");

    node.param("mode", _mode, (std::string)"UnknownPath");

    _lineInfoSub = node.subscribe(_lineInfoTopic, 1000, &Core::_lineInfoCb, this);
    _trafficSignsInfoSub = node.subscribe(_trafficSignsInfoTopic, 1000, &Core::_trafficSignsInfoCb, this);
    _trafficLightsInfoSub = node.subscribe(_trafficLightsInfoTopic, 1000, &Core::_trafficLightsInfoCb, this);

    _motorsInfoPub = node.advertise<aist::Motors>(_motorsInfoTopic, 1000);
    _lineControlPub = node.advertise<aist::LineControl>(_lineControlTopic, 1000);

    double P, I, D, minI, maxI;
    node.param("PID_P", P, 0.9);
    node.param("PID_I", I, 0.0);
    node.param("PID_D", D, 0.2);
    node.param("PID_minI", minI, -10.0);
    node.param("PID_maxI", maxI, 10.0);
    node.param("PID_B", _data.PID_B, 1.0);
    node.param("PID_K", _data.PID_K, 1.0);
    _data.pid = PID(P, I, D, minI, maxI);

    std::cout << "[PID]:" << " P: " << P << " I: " << I << " D: " << D << " B: " << _data.PID_B << " K: " << _data.PID_K << std::endl;

    if(_mode == "UnknownPath") {
        _stateMachine.push(std::make_shared<UnknownPathInitState>());
    } else if(_mode == "KnownPath") {
        _stateMachine.push(std::make_shared<KnownPathInitState>());
    } else if(_mode == "ProblematicRoute") {
        _stateMachine.push(std::make_shared<ProblematicRouteInitState>());
    } else if(_mode == "Qualification") {
        _stateMachine.push(std::make_shared<QualificationInitState>());
    }
}

void Core::update() {
    _stateMachine.update(_data);
    _publishInfo();
}

void Core::_lineInfoCb(const aist::LineInfo &msg) {
    _data.lostLine = msg.lost;
    _data.stopLine = msg.stopline;
    _data.forkLine = msg.fork;
    if(!_data.lockedSteering) {
        if (!msg.lost) {
            _data.steering = (float) _data.pid.calculate(msg.deviation, _data.steering);
        } else if(!_data.returnToLineAlgo) {
            _data.steering = 0;
        }
    }
}

void Core::_trafficSignsInfoCb(const aist::TrafficSignsInfo &msg) {
    for(const auto & sign : msg.appeared)
        _data.appearedSigns.push(sign);
    for(const auto & sign : msg.disappeared)
        _data.disappearedSigns.push(sign);
    _data.signs = msg.signs;
}

void Core::_trafficLightsInfoCb(const aist::TrafficLightsInfo &msg) {
    _data.trafficLights.push(msg);
}

void Core::_publishInfo() {
    _publishLineControl();
    _publishMotorsInfo();
}

void Core::_publishLineControl() {
    aist::LineControl msg;
    msg.header.frame_id = "line";
    msg.header.seq = _lineControlPubSeq++;
    msg.header.stamp = ros::Time::now();

    msg.direction = std::max((int8_t)-1, std::min((int8_t)1, _data.direction));

    _lineControlPub.publish(msg);
}

void Core::_publishMotorsInfo() {
    aist::Motors msg;
    msg.header.frame_id = "motors";
    msg.header.seq = _motorsInfoPubSeq++;
    msg.header.stamp = ros::Time::now();

    const double pi = atan(-1.0);
    const double torad = pi/180;

    msg.power = _data.power;

    //1
    msg.steering = _data.steering;
    //2
//    if(_data.steering == 90 || _data.steering == -90)
//        msg.steering = _data.steering;
//    else
//        msg.steering = (float)(atan(_data.PID_B*tan(_data.steering*_data.PID_K*torad))/(torad*_data.PID_K));
    //3
//    msg.steering = (float)(atan(_data.PID_B*tan(_data.steering*_data.PID_K*torad))/(torad*_data.PID_K));

    _motorsInfoPub.publish(msg);
}

Core::~Core() {
    std::cout << "[Core] Average line error " << _data.pid.getCost() << "\n";
}