#include <ros/ros.h>
#include "Runner.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "traffic_lights_nn_node");

    ros::NodeHandle node;

    Runner runner(node);

    ros::spin();
}