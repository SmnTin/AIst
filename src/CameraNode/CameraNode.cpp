#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <string>
#include <cmath>
#include <chrono>

image_transport::Subscriber sub;
image_transport::Publisher pub;

cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
cv::Mat distCoeffs;

cv::VideoCapture capture;

void load(const std::string & filename) {
    std::cout << filename << "\n";
    cv::FileStorage fIn(filename, cv::FileStorage::READ);
    fIn["intrinsic"] >> intrinsic;
    fIn["distCoeffs"] >> distCoeffs;
}

void openCapture(int deviceId, int width, int height) {
    if(!capture.open(deviceId))
        throw std::runtime_error("Can't open capture!");
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
}

void process(cv::Mat & in, cv::Mat & out) {
    in.copyTo(out);
//    undistort(in, out, intrinsic, distCoeffs);
}

void display(cv::Mat & in, cv::Mat & out) {
    cv::imshow("in", in);
    cv::imshow("out", out);
    cv::waitKey(30);
}

void onTick() {
    cv::Mat inImg, outImg;
    capture >> inImg;

    process(inImg, outImg);
//    display(inImg, outImg);

    sensor_msgs::ImagePtr response = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();
//    std::cout << "[Camera] " << response->header.seq << " " << (int)(std::chrono::steady_clock::now().time_since_epoch().count()/1000000.0f) << "\n";
    pub.publish(response);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "movement_node");
    ros::NodeHandle node("~");

    std::string outputImageTopic, calibrationFilePath;
    int framerate, deviceId, width, height;
    node.param("output_image_topic", outputImageTopic, (std::string)"/undistorted_image/image_raw");
    node.param("calibration_file_url", calibrationFilePath, (std::string)"");
    node.param("framerate", framerate, 30);
    node.param("device_id", deviceId, 0);
    node.param("width", width, 640);
    node.param("height", height, 480);

    load(calibrationFilePath);
    openCapture(deviceId, width, height);

    image_transport::ImageTransport it(node);
    pub = it.advertise(outputImageTopic, 1);

    ros::Rate rate(framerate);

    while(ros::ok()) {
        onTick();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}