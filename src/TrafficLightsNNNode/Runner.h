#ifndef TRAFFIC_LIGHTS_NODE_RUNNER_H
#define TRAFFIC_LIGHTS_NODE_RUNNER_H

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aist/TrafficLightsInfo.h>

#include "TrafficLightsDetector.h"

class Runner {
public:
    explicit Runner (ros::NodeHandle & node);

private:
    ros::NodeHandle & _node;

    int _cnt = 0;
    image_transport::ImageTransport * _imgTransport;
    image_transport::Subscriber _imageSub;

    std::string _imageTopic;
    std::string _trafficLightsInfoTopic;

    bool _displayingResult;

    void _imageCb(const sensor_msgs::ImageConstPtr & msg);
    void _parseImage(const sensor_msgs::ImageConstPtr &msg, cv::Mat & src);
    void _publishResult(const std::vector<TrafficLight> & info);
    void _displayResult(cv::Mat src, cv::Mat dst);

    TrafficLightsDetector _trafficLightsDetector;

    ros::Publisher _trafficLightsInfoPub;
    uint32_t _trafficLightsInfoPubSeq = 0;
};

#endif //TRAFFIC_LIGHTS_NODE_RUNNER_H
