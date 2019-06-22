#include "Runner.h"

Runner::Runner(ros::NodeHandle &node) : _node(node){
    node.param("image_topic", _imageTopic, (std::string)"/cv_camera_node/image_raw");
    node.param("traffic_signs_info_topic", _trafficSignsInfoTopic, (std::string)"/traffic_signs_info");
    node.param("display_result", _displayingResult, true);

    _imgTransport = new image_transport::ImageTransport(_node);
    _imageSub = _imgTransport->subscribe(_imageTopic, 1, &Runner::_imageCb, this);

    _trafficSignsInfoPub = node.advertise<aist::TrafficSignsInfo>(_trafficSignsInfoTopic, 1000);
}

void Runner::_imageCb(const sensor_msgs::ImageConstPtr &msg) {
    if((_cnt++)%3==0)
        return;
    cv::Mat src, dst;
    _parseImage(msg, src);
    src.copyTo(dst);

    TrafficSignsDetectorInfo curInfo = _trafficSignsDetector.detectTrafficSigns(src, dst);

    for(auto & sign : curInfo.appeared) {
        std::cout << "Appeared " << sign.type << " " << sign.rect << "\n";
    }
    for(auto & sign : curInfo.disappeared) {
        std::cout << "Disappeared " << sign.type << " " << sign.rect << "\n";
    }

    _publishResult(curInfo);
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

void Runner::_publishResult(const TrafficSignsDetectorInfo &info) {
    aist::TrafficSignsInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera";
    msg.header.seq = _trafficSignsInfoPubSeq++;

    _fillFrom(msg.signs, info.signs);
    _fillFrom(msg.appeared, info.appeared);
    _fillFrom(msg.disappeared, info.disappeared);

    _trafficSignsInfoPub.publish(msg);
}

void Runner::_fillFrom(std::vector<aist::TrafficSign> &to, const std::vector<TrafficSign> &from) {
    for(const auto & sign : from) {
        aist::TrafficSign msgSign;
        msgSign.type = sign.type;
        msgSign.distance = sign.distance;
        msgSign.lastSeen.data = sign.lastAppeared;
        msgSign.position.x = sign.rect.x;
        msgSign.position.y = sign.rect.y;
        msgSign.size.x = sign.rect.width;
        msgSign.size.y = sign.rect.height;
        to.push_back(msgSign);
    }
}