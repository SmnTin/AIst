#include "Runner.h"

Runner::Runner(ros::NodeHandle &node) : _node(node){
    node.param("image_topic", _imageTopic, (std::string)"/cv_camera_node/image_raw");
    node.param("line_info_topic", _lineInfoTopic, (std::string)"/line_info");
    node.param("line_control_topic", _lineControlTopic, (std::string)"/line_control");
    node.param("display_result", _displayingResult, true);

    _imgTransport = new image_transport::ImageTransport(_node);
    _imageSub = _imgTransport->subscribe(_imageTopic, 1, &Runner::_imageCb, this);

    _lineControlSub = node.subscribe(_lineControlTopic, 1000, &Runner::_lineControlCb, this);
    _lineInfoPub = node.advertise<aist::LineInfo>(_lineInfoTopic, 1000);

    _lineDetector = new LineDetector(!_displayingResult);
    _lineDetector->setDirection(0);
}

void Runner::_lineControlCb(const aist::LineControl &msg) {
//    ROS_INFO("Direction: %d", msg.direction);
    _lineDetector->setDirection(msg.direction);
}

void Runner::_imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat src, dst;
    _parseImage(msg, src);
    src.copyTo(dst);

    LineDetectorInfo curInfo = _lineDetector->detectLine(src, dst);
    int16_t oldDeviation = _info.oldDeviation;
    const float filterCoef = 0.8f;
//    _info.deviation = filterCoef*oldDeviation + (1-filterCoef)*_info.deviation;
    if(!_info.lost)
        oldDeviation = (int16_t)_info.deviation;
    _info = curInfo;
    _info.oldDeviation = oldDeviation;

    _publishResult(_info);
    if(_displayingResult)
        _displayResult(src, dst);
}

void Runner::_parseImage(const sensor_msgs::ImageConstPtr &msg, cv::Mat & src) {
    cv_bridge::CvImageConstPtr cvImg;
    try
    {
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            cvImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        else
            cvImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cvImg->image.copyTo(src);
}

void Runner::_displayResult(cv::Mat src, cv::Mat dst) {
//    cv::imshow("src", src);
    cv::imshow("dst", dst);
    cv::waitKey(30);
}

void Runner::_publishResult(const LineDetectorInfo &info) {
    aist::LineInfo msgPub;
    msgPub.fork = info.fork;
    msgPub.lost = info.lost;
    msgPub.stopline = info.stopline;
    msgPub.deviation = info.deviation;
    msgPub.oldDeviation = info.oldDeviation;
    msgPub.deviation2 = info.deviation2;
    msgPub.header.stamp = ros::Time::now();
    msgPub.header.frame_id = "camera";
    msgPub.header.seq = _lineInfoPubSeq++;

    _lineInfoPub.publish(msgPub);
}

Runner::~Runner() {
    delete _lineDetector;
}