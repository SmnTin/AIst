#ifndef CORE_NODE_CORE_H
#define CORE_NODE_CORE_H

#include <ros/ros.h>
#include "Data.h"

#include <string>
#include <cmath>

#include "StateMachine.h"

class Core {
public:
    explicit Core(ros::NodeHandle & node);
    void update();
    ~Core();

private:
    ros::NodeHandle & _node;

    std::string _lineInfoTopic;
    std::string _trafficSignsInfoTopic;
    std::string _trafficLightsInfoTopic;
    std::string _lineControlTopic;
    std::string _motorsInfoTopic;

    std::string _mode;

    ros::Subscriber _trafficSignsInfoSub;
    ros::Subscriber _trafficLightsInfoSub;

    ros::Subscriber _lineInfoSub;
    ros::Publisher _lineControlPub;
    uint32_t _lineControlPubSeq = 0;

    ros::Publisher _motorsInfoPub;
    uint32_t _motorsInfoPubSeq = 0;

    Data _data;

    StateMachine<Data> _stateMachine;

    void _lineInfoCb(const aist::LineInfo & msg);
    void _trafficSignsInfoCb(const aist::TrafficSignsInfo & msg);
    void _trafficLightsInfoCb(const aist::TrafficLightsInfo & msg);

    void _publishInfo();
    void _publishLineControl();
    void _publishMotorsInfo();
};

#endif //CORE_NODE_CORE_H
