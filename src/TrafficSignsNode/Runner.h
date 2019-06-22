#ifndef TRAFFIC_SIGNS_NODE_RUNNER_H
#define TRAFFIC_SIGNS_NODE_RUNNER_H

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aist/TrafficSignsInfo.h>

#include "TrafficSignsDetector.h"

class Runner {
public:
    explicit Runner (ros::NodeHandle & node);

private:
    ros::NodeHandle & _node;

    int _cnt = 0;

    image_transport::ImageTransport * _imgTransport;
    image_transport::Subscriber _imageSub;

    std::string _imageTopic;
    std::string _trafficSignsInfoTopic;

    bool _displayingResult;

    void _imageCb(const sensor_msgs::ImageConstPtr & msg);
    void _parseImage(const sensor_msgs::ImageConstPtr &msg, cv::Mat & src);
    void _publishResult(const TrafficSignsDetectorInfo & info);
    void _fillFrom(std::vector<aist::TrafficSign> & to, const std::vector<TrafficSign> & from);
    void _displayResult(cv::Mat src, cv::Mat dst);

    TrafficSignsDetector _trafficSignsDetector;
    TrafficSignsDetectorInfo _info;

    ros::Publisher _trafficSignsInfoPub;
    uint32_t _trafficSignsInfoPubSeq = 0;
};

#endif //TRAFFIC_SIGNS_NODE_RUNNER_H
