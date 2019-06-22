#ifndef LINE_NODE_RUNNER_H
#define LINE_NODE_RUNNER_H

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aist/LineInfo.h>
#include <aist/LineControl.h>

#include "LineDetector.h"

class Runner {
public:
    explicit Runner (ros::NodeHandle & node);
    ~Runner();

private:
    ros::NodeHandle & _node;

    image_transport::ImageTransport * _imgTransport;
    image_transport::Subscriber _imageSub;

    std::string _imageTopic;
    std::string _lineInfoTopic;
    std::string _lineControlTopic;

    bool _displayingResult;

    void _lineControlCb(const aist::LineControl & msg);

    void _imageCb(const sensor_msgs::ImageConstPtr & msg);
    void _parseImage(const sensor_msgs::ImageConstPtr &msg, cv::Mat & src);
    void _publishResult(const LineDetectorInfo & info);
    void _displayResult(cv::Mat src, cv::Mat dst);

    LineDetector * _lineDetector;
    LineDetectorInfo _info;

    ros::Subscriber _lineControlSub;
    ros::Publisher _lineInfoPub;
    uint32_t _lineInfoPubSeq = 0;
};

#endif //PROJECT_RUNNER_H
