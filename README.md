# AIst

The software for an autonomous robot based on [NVidia Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2) running [ROS](https://www.ros.org/). The robot can move along the black line, and recognize signs and traffic lights. The behaviour is controlled by a stack-based state machine.

![The robot](robot.jpg)

## Hardware

* [NVidia Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2)
* USB camera with 120° FOV + USB hub
* [PCA9685 PWM driver](https://www.adafruit.com/product/815)
* Front-wheels servo motor
* Rear-wheels DC motor + PWM driver
* [Team Associated B5 Buggy](https://www.associatedelectrics.com/teamassociated/cars_and_trucks/RC10B5/Team/)
* 2x [LIPO Turnigy nano-tech 5800 mah](https://hobbyking.com/en_us/turnigy-nano-tech-5800mah-2s2p-30-60c-hardcase-lipo-pack-roar-approved.html)

Jetson talks to the motors via the PWM driver.

## Software

The software is written in C++. It is split into several standalone [ROS](https://www.ros.org/) nodes. The recognition part is done using [OpenCV](https://opencv.org/).

* The algorithm for line recognition is able to build a tree along the line and detect forks and crossings.
![Line detection](line-detection.jpg)
* For sign recognition we use Hoare cascade classifier.
* For traffic light recognition we use a neural network with a multi-layer perceptron architecture.
* To contol the robot we use a stack-based state machine.
