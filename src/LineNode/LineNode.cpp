#include <ros/ros.h>
#include "Runner.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle node;

    Runner runner(node);

    ros::spin();
}