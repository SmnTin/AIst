#include "TrafficLightsDetector.h"

TrafficLightsDetector::TrafficLightsDetector(int window)
         : _window(window) {
    std::string path = ros::package::getPath("aist");
    for(int i=0;i<N;i++) {
        if(!nn[i].loadModel((path + "/assets/model_tmp.mdl").c_str())) {
            printf("[E]: Can't load model\n");
        }
    }
    signCode[0] = sign_stop;
    signCode[1] = sign_crosswalk;
    signCode[2] = sign_giveway;
    signCode[3] = sign_mainroad;
}

std::vector<TrafficLight> TrafficLightsDetector::detectTrafficLights(cv::Mat src, cv::Mat drawing) {
    auto lights = _detect(src);
    _draw(lights, drawing);

    return lights;
}

std::vector<TrafficLight> TrafficLightsDetector::_detect(cv::Mat src) {
    cv::Rect bigRoiRect(0, (src.rows - _window) / 2, src.cols, _window);
    cv::Mat bigRoi = src(bigRoiRect).clone();

    std::vector<TrafficLight> lights;

    cv::Mat gray0[3];
    cv::Mat pyr, timg;

    resize(bigRoi, pyr, cv::Size(bigRoi.cols/2, bigRoi.rows/2));
    resize(pyr, timg, bigRoi.size());

    split(timg,gray0);

    const int threshCoef = 50;

    for( int c = 0; c < 3; c++ )
    {
#pragma omp parallel for
        for( int l = 0; l < N; l++ )
        {
            cv::Mat gray;
            if( l == 0 )
            {
                cv::Canny(gray0[c], gray, 0, threshCoef, 5);
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                gray = gray0[c] >= (l+1)*255/N;
            }

            std::vector<std::vector<cv::Point> > contours;
            findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;

            for( size_t i = 0; i < contours.size(); i++ )
            {
                approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);

                double area = fabs(contourArea(cv::Mat(approx)));

                if(area > 200 && area < 10000 && isContourConvex(cv::Mat(approx)))
                {
                    cv::Rect boundingarea = boundingRect(approx);


//                    if(boundingarea.width>120 || boundingarea.height>120) continue; //
                    if(boundingarea.width<13 || boundingarea.height<13) continue;
                    double dl = (double)boundingarea.width/(double)boundingarea.height;
                    if(dl<0.3 || dl>1.2) continue;

                    cv::Mat sng;
                    cv::Mat sng_t = bigRoi(boundingarea);
                    simple_hist colors;
                    color_counter(&sng_t, &colors);
                    cvtColor(sng_t,sng, CV_BGR2GRAY);
                    resize(sng,sng,cv::Size(16,16));
                    double inputs[16*16];
                    for(int  j=0; j< sng.rows*sng.cols; j++) {
                        inputs[j] = (((double)sng.data[j])/255.0);
                    }

                    double answers[8];
                    nn[l].calculate(inputs,answers);

                    int maxx = 0;
                    for(int j=1;j<9;j++) {
                        if(answers[j]>answers[maxx]) maxx=j;
                    }

                    if(answers[maxx]>0.85) {
                        TrafficLight mysign;
                        mysign.type = -1;
                        if(maxx >= 4) {
                            if(answers[4]>0.9) {
                                if(dl<0.8 && dl>0.3) {
                                    if(answers[5]>0.8) mysign.type = 0;
                                    else if(answers[6]>0.8) mysign.type = 2;
                                    else if(answers[7]>0.6) mysign.type = 3;
                                    mysign.rect = boundingarea;
                                    mysign.distance = (_focalLength * _trafficLightWidth) / boundingarea.width;
                                    _addOffset(mysign.rect, cv::Point(0, (src.rows - _window) / 2));
                                    if(mysign.type != -1)
                                        lights.push_back(mysign);
                                }
                            }
                        }

//                        ROS_INFO("%d %lf %lf %lf %lf %d %lf\n", maxx, answers[4], answers[5], answers[6], answers[7], mysign.type, dl);

                    }
//                    if(answers[4] > 0.1)
//                        ROS_INFO("%lf\n", answers[4]);
                }
            }
        }
    }
    return lights;
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

void TrafficLightsDetector::_addOffset(cv::Rect & rect, cv::Point offset) {
    rect = cv::Rect(rect.x + offset.x, rect.y + offset.y, rect.width, rect.height);
}