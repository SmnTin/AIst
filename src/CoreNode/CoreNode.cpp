#include <ros/ros.h>
#include "Core.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "core_node");

    ros::NodeHandle node("~");

    Core core(node);

    ros::Rate rate(30);
    while(ros::ok()) {
        core.update();
        ros::spinOnce();
        rate.sleep();
    }
}