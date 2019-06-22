#include "Runner.h"

Runner::Runner(ros::NodeHandle &node) : _node(node){
    node.param("image_topic", _imageTopic, (std::string)"/cv_camera_node/image_raw");
    node.param("traffic_lights_info_topic", _trafficLightsInfoTopic, (std::string)"/traffic_lights_info");
    node.param("display_result", _displayingResult, true);

    _imgTransport = new image_transport::ImageTransport(_node);
    _imageSub = _imgTransport->subscribe(_imageTopic, 1, &Runner::_imageCb, this);

    _trafficLightsInfoPub = node.advertise<aist::TrafficLightsInfo>(_trafficLightsInfoTopic, 1000);
}

void Runner::_imageCb(const sensor_msgs::ImageConstPtr &msg) {
    if((_cnt++)%3==0)
        return;
    cv::Mat src, dst;
    _parseImage(msg, src);
    src.copyTo(dst);

    auto curInfo = _trafficLightsDetector.detectTrafficLights(src, dst);

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

void Runner::_publishResult(const std::vector<TrafficLight> &info) {
    for(const auto & light : info) {
        aist::TrafficLightsInfo msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "camera";
        msg.header.seq = _trafficLightsInfoPubSeq++;

        msg.type = light.type;
        msg.starting = light.starting;
        msg.distance = light.distance;

        _trafficLightsInfoPub.publish(msg);
    }
}