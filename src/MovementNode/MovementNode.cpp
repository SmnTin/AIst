#include <ros/ros.h>
#include <aist/Motors.h>

#include "JHPWMPCA9685.h"

#include <string>
#include <cmath>
#include <chrono>

PCA9685 *pca9685;

const int servoZero = 86;
const int servoMin = 256;
const int servoMax = 620;
const int servoAngMin = 0;
const int servoAngMax = 180;

//const int servoZero = 99;
//const int servoMin = 120;
//const int servoMax = 720;
//const int servoAngMin = 20;
//const int servoAngMax = 160;

const int motorMin = 0;
const int motorMax = 2000;

const int servoChannel = 4;
const int motorChannel = 3;

double map ( double x, double in_min, double in_max, double out_min, double out_max) {
    double toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

void init() {
    pca9685 = new PCA9685(0x70);
    pca9685->kI2CBus = 1;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d\n", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0);
        pca9685->reset();
        pca9685->setPWMFrequency(60);
        pca9685->setPWM(servoChannel,0,servoMin);
        pca9685->setPWM(motorChannel,0,motorMin);
    }
}

void writePWM(int channel, int value) {
    pca9685->setPWM(channel, 0, value);
//    std::cout << "[Movement]" << channel << " " << value << "\n";
}

void stop() {
    writePWM(servoChannel, (int)map(servoZero, 0, 180, servoMin, servoMax));
    writePWM(motorChannel, motorMin);
    pca9685->closePCA9685();
}

bool motorsSwitchState = false;
time_t lastSwitch = time(nullptr);
time_t switchEvery = 2;

void motorsInfoCb(const aist::Motors & msg) {
//    std::cout << "[Movement] " << msg.header.seq << " " << (int)(std::chrono::steady_clock::now().time_since_epoch().count()/1000000.0f) << "\n";
//    int steering = std::max(servoAngMin, std::min(servoAngMax, (int)map(-(int)lround(0) + servoZero, 0, 180, servoAngMin, servoAngMax)));
//    int steering = std::max(servoAngMin, std::min(servoAngMax, (int)map(-(int)lround(msg.steering) + servoZero, 0, 180, servoAngMin, servoAngMax)));
    int steering = std::max(servoAngMin, std::min(servoAngMax, (int)map(-(int)lround(msg.steering), -90, 90, servoAngMin-90, servoAngMax-90) + servoZero));
//    int steering = std::max(servoAngMin, std::min(servoAngMax, (int)lround(msg.steering) + servoZero));
//    int steering = servoZero;
    int stPWM = (int)map(steering, 0, 180, servoMin, servoMax);
    double power = msg.power;
    int powerPWM;

    if(time(nullptr) - lastSwitch > switchEvery) {
        motorsSwitchState = !motorsSwitchState;
        lastSwitch = time(nullptr);
    }

//    if(motorsSwitchState)
//        powerPWM = (int)map(power, 0, 1, motorMin, motorMax) - 20;
//    else
    powerPWM = (int)map(power, 0, 1, motorMin, motorMax);
    writePWM(servoChannel, stPWM);
    writePWM(motorChannel, powerPWM);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "movement_node");
    ros::NodeHandle node;
    srand(time(0));

    std::string motorsInfoTopic;
    node.param("motors_info_topic", motorsInfoTopic, (std::string)"/motors_info");
    ros::Subscriber motorsInfoSub = node.subscribe(motorsInfoTopic, 1000, motorsInfoCb);

    init();
    ros::spin();
    stop();

    return 0;
}