#ifndef PROJECT_TRAFFICSIGNSDETECTOR_H
#define PROJECT_TRAFFICSIGNSDETECTOR_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/utils/trace.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <utility>

struct TrafficSign {
    int type = 0;
    double distance = 0;
    ros::Time firstAppeared, lastAppeared;
    ros::Duration existenceTime;
    cv::Rect rect;
    bool _appeared = false, _live = false, _merged = false;
};

struct TrafficSignsDetectorInfo {
    std::vector<TrafficSign> signs, appeared, disappeared;
};

class TrafficSignsDetector {
public:
    explicit TrafficSignsDetector(int window = 150);
    TrafficSignsDetectorInfo detectTrafficSigns(cv::Mat src, cv::Mat drawing);

private:
    int _window;
    const double _timeToDisappear = 0.8;
    const double _timeToAppear = 0.8;
    const double _mergeDist = 100.0;
    const double _focalLength = 330;
    const double _signWidth = 0.07;

    std::vector<TrafficSign> _detect(cv::Mat src);
    void _draw(cv::Mat drawing);
    TrafficSignsDetectorInfo _trackChanges(const std::vector<TrafficSign> & curSigns);
    double _offset(cv::Rect a, cv::Rect b);
    std::vector<TrafficSign> _signs;
    std::vector<std::string> _files;
    std::vector<cv::CascadeClassifier> _cascades;
    std::vector<int> _KNN;
    std::string _path;
};

#endif //PROJECT_TRAFFICSIGNSDETECTOR_H
