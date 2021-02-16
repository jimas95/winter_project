#include "ros/ros.h"
#include "commander/commandSRV.h"
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

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

// convert yaw to Quaternion
geometry_msgs::Quaternion get_quad(TargetLocation target){
    tf2::Quaternion q;
    q.setRPY(0, 0, target.orientation);
    geometry_msgs::Quaternion quad_msg;
    quad_msg = tf2::toMsg(q);
    return quad_msg;
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

    goal.target_pose.pose.position.x = target.x;
    goal.target_pose.pose.position.y = target.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = get_quad(target);

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Commander");
    ros::NodeHandle n;
    
    // Services 
    ros::ServiceServer service = n.advertiseService("commander", get_command);
    ros::Publisher cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);

    // tf for position 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    // ros::spin();
    auto frequency = 5;
    ros::Rate loop_rate(frequency);
    TargetLocation kitchen_loc{0,0,0};
    TargetLocation shelfs_loc{1,0,0};
    TargetLocation home_loc{1,1,0};
    TargetLocation drawers_loc{2,2,0};

    ROS_INFO("Ready to Recieve Commands!");

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
                    set_goal(kitchen_loc);
                    break;
                case Position::shelfs:
                    set_goal(shelfs_loc);
                    break;
                case Position::home:
                    set_goal(home_loc);
                    break;
                case Position::drawers:
                    set_goal(drawers_loc);
                    break;
                default : 
                    ROS_ERROR("Unknow rec_command");
            }   
            rec_command = false;
        }


        geometry_msgs::TransformStamped transformStamped;
        try{
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint",ros::Time(0));
        ROS_INFO_STREAM("robot x --> " << transformStamped.transform.translation.x);
        ROS_INFO_STREAM("robot y --> " << transformStamped.transform.translation.y);

        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
        }

        loop_rate.sleep();


        
    }

    return 0;
}



