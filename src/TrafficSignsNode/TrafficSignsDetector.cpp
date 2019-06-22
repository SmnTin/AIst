#include "TrafficSignsDetector.h"

#include <iostream>

TrafficSignsDetector::TrafficSignsDetector(int window) {
    _window = window;
    _path = ros::package::getPath("aist");
//    std::cout << _path << "\n";
    _files = {
            "/assets/haar/stop_cascade.xml",
            "/assets/haar/left_cascade.xml",
            "/assets/haar/right_cascade.xml",
            "/assets/haar/ped_cascade.xml",
            "/assets/haar/curvy_cascade.xml",
            "/assets/haar/pass_left_cascade.xml"
    };

    _KNN = {1, 7, 7, 7, 2, 7};

    for(std::string & s : _files) {
        std::cout << _path + s << "\n";
        cv::CascadeClassifier cascade;
        if(!cascade.load(_path + s))
            std::cout << "Error loading " << s << "\n";
        _cascades.emplace_back(cascade);
    }
}

TrafficSignsDetectorInfo TrafficSignsDetector::detectTrafficSigns(cv::Mat src, cv::Mat drawing) {
    std::vector<TrafficSign> curSigns = _detect(src);

    TrafficSignsDetectorInfo info = _trackChanges(curSigns);

    _draw(drawing);

    return info;
}

void TrafficSignsDetector::_draw(cv::Mat drawing) {
    cv::rectangle(drawing, cv::Rect(0, (drawing.rows - _window) / 2, drawing.cols, _window), cv::Scalar(255,0,0));

    for(auto & sign : _signs) {
        if(!sign._appeared)
            continue;
        cv::rectangle(drawing, sign.rect, cv::Scalar(0, 255, 0));
        putText(
                drawing,
                std::to_string(sign.type) + " " + std::to_string(sign.distance),
                cv::Point(sign.rect.x, sign.rect.y),
                0, 0.5,
                cv::Scalar(244,0,0),
                1, 8, false);
    }
}

std::vector<TrafficSign> TrafficSignsDetector::_detect(cv::Mat src) {
    cv::Rect bigRoiRect(0, (src.rows - _window) / 2, src.cols, _window);
    cv::Mat bigRoi = src(bigRoiRect).clone();

    cv::Mat bigRoiGray;
    cv::cvtColor( bigRoi, bigRoiGray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( bigRoiGray, bigRoiGray );

    std::vector<TrafficSign> signs;
    for(int i = 0; i < _cascades.size(); ++i) {
        auto & cascade = _cascades[i];
        std::vector<cv::Rect> curSigns;
        cascade.detectMultiScale( bigRoiGray, curSigns, 1.1, _KNN[i], (0|cv::CASCADE_SCALE_IMAGE), cv::Size(30, 30));

        for(auto & rect : curSigns) {
            TrafficSign sign;
            sign.type = i;
            sign.distance = (_focalLength * _signWidth) / std::max(rect.width, rect.height);
            sign.firstAppeared = ros::Time::now();
            sign.lastAppeared = ros::Time::now();
            sign.rect = cv::Rect(rect.x, rect.y + (src.rows - _window) / 2, rect.width, rect.height);
            signs.push_back(sign);
        }
    }

    return signs;
}

TrafficSignsDetectorInfo TrafficSignsDetector::_trackChanges(const std::vector<TrafficSign> & curSigns) {
    TrafficSignsDetectorInfo info;

    std::vector<TrafficSign> appeared, signs, disappeared;

    for(TrafficSign & _sign : _signs) {
        _sign._merged = false;
    }

    for(const TrafficSign & sign : curSigns) {
        bool found = false;
        for(TrafficSign & _sign : _signs) {
            if(sign.type == _sign.type && _offset(sign.rect, _sign.rect) <= _mergeDist) {
                if(_sign._live)
                    _sign.existenceTime += sign.lastAppeared - _sign.lastAppeared;
                else
                    _sign._live = true;
                _sign._merged = true;
                _sign.lastAppeared = sign.lastAppeared;
                _sign.distance = sign.distance;
                _sign.rect = sign.rect;
                found = true;
            }
        }
        if(!found) {
            signs.push_back(sign);
        }
    }

    for(TrafficSign & _sign : _signs) {
        if(!_sign._merged)
            _sign._live = false;
    }

    for(TrafficSign & _sign : _signs) {
        if(!_sign._appeared && _sign.existenceTime.toSec() >= _timeToAppear) {
            appeared.push_back(_sign);
            _sign._appeared = true;
        }
    }

    for(TrafficSign & _sign : _signs) {
        if((ros::Time::now() - _sign.lastAppeared).toSec() >= _timeToDisappear && _sign._appeared)
            disappeared.push_back(_sign);
        else
            signs.push_back(_sign);
    }

    _signs = signs;
    info.appeared = appeared;
    info.disappeared = disappeared;
    for(auto & _sign : _signs)
        if(_sign._appeared)
            info.signs.push_back(_sign);

    return info;
}

double TrafficSignsDetector::_offset(cv::Rect a, cv::Rect b) {
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}