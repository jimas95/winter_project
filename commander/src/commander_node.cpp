#include "ros/ros.h"
#include "commander/commandSRV.h"
#include <iostream>
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Point.h"
// #include "geometry_msgs/Quaternion.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <array>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




enum Position {abandon, kitchen, shelfs, home, drawers };
Position map_position = home;
static bool rec_command = false;

struct TargetLocation{
    double x = 0;
    double y = 0;
    double orientation = 0;

};

// need robots position
// cancle goal
// set goal
// rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}


std::string getEnumName(Position map_position ){
    switch(map_position){
        case Position::abandon:
            return "abandon";
        case Position::kitchen:
            return "kitchen";
        case Position::shelfs:
            return "shelfs";
        case Position::home:
            return "home";
        case Position::drawers:
            return "drawers";
        default : 
            ROS_ERROR("Unknow Position");
            return "---";
    }   
}

bool get_command(commander::commandSRV::Request  &req,
         commander::commandSRV::Response &res)
{
    rec_command = true;
    switch(req.input_command){
        case 0:
            map_position = Position::abandon;
            res.result = "Recieved Command --> abandon";
            break;
        case 1:
            map_position = Position::home;
            res.result = "Recieved Command --> home";
            break;
        case 2:
            map_position = Position::kitchen;
            res.result = "Recieved Command --> kitchen";
            break;
        case 3:
            map_position = Position::shelfs;
            res.result = "Recieved Command --> shelfs";
            break;
        case 4:
            map_position = Position::drawers;
            res.result = "Recieved Command --> drawers";
            break;
        default : 
            ROS_ERROR("service command --> Unknow Command");
            res.result = "Unknow Command";
            rec_command = false;
            return false;

    }   
    ROS_INFO_STREAM(res.result);
    return true;
}


// set goal position for robot, 
// Input target --> goal to go
void set_goal(TargetLocation target){
    

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);


    // //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map"; // these need to change
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Commander");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("commander", get_command);
    ros::Publisher cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);


    ROS_INFO("Ready to Recieve Commands!");


    // ros::spin();
    auto frequency = 5;
    ros::Rate loop_rate(frequency);
    TargetLocation kitchen{0,0,0};
    TargetLocation home{0,0,0};

    while(ros::ok()){
        ROS_INFO_STREAM("Status --> " << getEnumName(map_position));
        ros::spinOnce();

        if(rec_command){

            switch(map_position){
                case Position::abandon:
                    {
                        actionlib_msgs::GoalID msg;
                        msg.id = "";
                        cancel_pub.publish(msg);
                    }
                    
                    break;
                case Position::kitchen:
                    break;
                case Position::shelfs:
                    break;
                case Position::home:
                    set_goal(home);
                    break;
                case Position::drawers:
                    break;
                default : 
                    ROS_ERROR("Unknow rec_command");
            }   
            rec_command = false;
        }

        loop_rate.sleep();


        
    }

    return 0;
}



