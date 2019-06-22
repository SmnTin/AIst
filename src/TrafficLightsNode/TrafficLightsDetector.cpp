#include "TrafficLightsDetector.h"

TrafficLightsDetector::TrafficLightsDetector(int window)
         : _window(window) {}

std::vector<TrafficLight> TrafficLightsDetector::detectTrafficLights(cv::Mat src, cv::Mat drawing) {
    auto lights = _detect(src);
    _draw(lights, drawing);

    return lights;
}

std::vector<TrafficLight> TrafficLightsDetector::_detect(cv::Mat src) {
    cv::Rect bigRoiRect(0, (src.rows - _window) / 2, src.cols, _window);
    cv::Mat bigRoi = src(bigRoiRect).clone();

    std::vector<TrafficLight> lights;

    for(int thr = 90; thr <= 200; thr += 10) {
        cv::Mat threshMat;
        _preprocessImg(bigRoi, threshMat, thr);
//        cv::imshow((std::string)"thresh"+std::to_string(thr), threshMat);

//    cv::imshow("thr", threshMat);

        std::vector<cv::Rect> rects;
        _extractRects(threshMat, rects);

//    cv::Mat rectsMat = src.clone();

        for (auto &rect : rects) {

            double ratio = (double) rect.height / rect.width;
            TrafficLight light;
            if (ratio > 2)
                light = _classifyBig(threshMat(rect));
            else
                light = _classifySmall(threshMat(rect));
            light.rect = rect;
            light.distance = (_focalLength * _trafficLightWidth) / rect.width;
            _addOffset(light.rect, cv::Point(0, (src.rows - _window) / 2));

            if (light.type > -1)
                lights.push_back(light);
        }

    }
    return lights;
}

void TrafficLightsDetector::_preprocessImg(cv::Mat src, cv::Mat & dst, int thr) {
    cv::Mat greyMat, bluredMat, threshMat, erodedMat, dilatedMat;

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * _erosionSize + 1, 2 * _erosionSize + 1), cv::Point(_erosionSize, _erosionSize) );

    cv::cvtColor(src, greyMat, cv::COLOR_BGR2GRAY);
    cv::equalizeHist( greyMat, greyMat );

    cv::GaussianBlur( greyMat, bluredMat, cv::Size( 9, 9 ), 0, 0 );

    cv::threshold(bluredMat, threshMat, thr, 255, cv::THRESH_BINARY_INV);
//    cv::adaptiveThreshold(bluredMat, threshMat, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2);

    //Morphology
    cv::erode(threshMat,erodedMat,element);
    cv::dilate(erodedMat,dilatedMat,element);
    cv::dilate(erodedMat,dilatedMat,element);
    cv::erode(dilatedMat,dst,element);
}

void TrafficLightsDetector::_extractRects(cv::Mat thresh, std::vector<cv::Rect> & rects) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours(thresh, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for( auto & contour : contours ) {
        cv::Rect bound = boundingRect(contour);

        double ratio = (double)bound.height / bound.width;
        double area = bound.height * bound.width;
        double areaRatio = cv::contourArea(contour) / area;

        if(areaRatio > 0.8 && ratio > 1.5 && ratio < 3 && area > 500)
            rects.push_back(bound);
    }
}

void TrafficLightsDetector::_addOffset(cv::Rect & rect, cv::Point offset) {
    rect = cv::Rect(rect.x + offset.x, rect.y + offset.y, rect.width, rect.height);
}

TrafficLight TrafficLightsDetector::_classifySmall(cv::Mat src) {
    double red   = _cnt(src(cv::Rect(0, 0,            src.cols, src.rows / 2)));
    double green = _cnt(src(cv::Rect(0, src.rows / 2, src.cols, src.rows / 2)));

    TrafficLight res;
    res.starting = true;

    if(red > lightCoef && green < lightCoef2) {
        res.type = 0;
    } else if(green > lightCoef && red < lightCoef2){
        res.type = 3;
    }

//    std::cout << red << " " << green << " " << res.type << "\n";

    return res;
}

TrafficLight TrafficLightsDetector::_classifyBig(cv::Mat src) {
    double red   = _cnt(src(cv::Rect(0, 0,                src.cols, src.rows / 3)));
    double amber = _cnt(src(cv::Rect(0, src.rows / 3,     src.cols, src.rows / 3)));
    double green = _cnt(src(cv::Rect(0, src.rows / 3 * 2, src.cols, src.rows / 3)));

    TrafficLight res;
    res.starting = true;

    if(red > lightCoef && amber > lightCoef && green < lightCoef2) {
        res.type = 1;
    } else if(red > lightCoef && amber < lightCoef2 && green < lightCoef2) {
        res.type = 0;
    } else if(amber > lightCoef && green < lightCoef2 && red < lightCoef2) {
        res.type = 2;
    } else if(green > lightCoef && amber < lightCoef2 && red < lightCoef2){
        res.type = 3;
    }

    return res;
}

double TrafficLightsDetector::_cnt(cv::Mat src) {
//    cv::imshow("lig", src);

    int cnt = 0;
    for(int i = 0; i < src.rows; ++i)
        if((int)src.at<unsigned char>(cv::Point(src.cols/2, i)) == 0)
            cnt++;

    return (double)cnt / src.rows;
}

void TrafficLightsDetector::_draw(const std::vector<TrafficLight> & lights, cv::Mat drawing) {
    cv::rectangle(drawing, cv::Rect(0, (drawing.rows - _window) / 2, drawing.cols, _window), cv::Scalar(255,0,255));

    for(const auto & light : lights) {
        cv::Scalar color;
        switch (light.type) {
            default:
            case 0:
                color = cv::Scalar(0, 0, 255);
                break;
            case 1:
                color = cv::Scalar(0, 102, 255);
                break;
            case 2:
                color = cv::Scalar(0, 191, 255);
                break;
            case 3:
                color = cv::Scalar(0, 255, 0);
                break;
        }
        cv::rectangle(drawing, light.rect, color);
//    putText(
//            drawing,
//            std::to_string(sign.type),
//            cv::Point(sign.rect.x, sign.rect.y),
//            0, 0.5,
//            cv::Scalar(244,0,0),
//            1, 8, false);
    }
}