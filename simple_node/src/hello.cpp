#include <ros/ros.h>
int main (int argc, char **argv)
{
    ros::init(argc, argv, "roscpp_rate_test");
    ros::NodeHandle nh;
    ros::Rate rate(5); // ROS Rate at 5Hz
    while (ros::ok()) {
        ROS_INFO("Hello");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}