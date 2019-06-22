#ifndef PROJECT_TRAFFICLIGHTSDETECTOR_H
#define PROJECT_TRAFFICLIGHTSDETECTOR_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include "net.hpp"
#include "signs.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>

struct TrafficLight {
    int type = -1;
    double distance = 0;
    bool starting = false;
    cv::Rect rect;
};

class TrafficLightsDetector {
public:
    explicit TrafficLightsDetector(int window = 150);
    std::vector<TrafficLight> detectTrafficLights(cv::Mat src, cv::Mat drawing);

private:
    int _window;
    const int _erosionSize = 3;
    const double lightCoef = 0.5;
    const double lightCoef2 = 0.1;
    const int N = 11;
    NetDP nn[11];
    signs signCode[6];

    const double _trafficLightWidth = 0.05;
    const double _focalLength = 330;

    std::vector<TrafficLight> _detect(cv::Mat src);
    void _draw(const std::vector<TrafficLight> & lights, cv::Mat drawing);
    void _addOffset(cv::Rect & rect, cv::Point offset);
};

#endif //PROJECT_TRAFFICLIGHTSDETECTOR_H
