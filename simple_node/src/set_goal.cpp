#include "ros/ros.h"
// service of setting goal (kitchen,selfs,drawers)
// service of canceling goal 
// struct of goals


int main (int argc, char **argv)
{
    ros::init(argc, argv, "roscpp_rate_test");
    ros::NodeHandle nh;
    ros::Rate rate(5); // ROS Rate at 5Hz
    while (ros::ok()) {
        ROS_INFO("Lets Set a Goal!");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}