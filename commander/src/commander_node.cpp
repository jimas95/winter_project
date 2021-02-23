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
#define PI 3.14159265359

//why not stop ?
//no move_base action ? 
// installation folders

enum Position {abandon, kitchen, shelfs, home, drawers, patrol,idle };
Position map_position = idle;
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
        case Position::patrol:
            return "patrol";
        case Position::idle:
            return "idle";
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
        case 5:
            map_position = Position::patrol;
            res.result = "Recieved Command --> patrol";
            break;
        case 6:
            map_position = Position::idle;
            res.result = "Recieved Command --> idle";
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

move_base_msgs::MoveBaseGoal create_goal(std::string frame_id,TargetLocation target){
    move_base_msgs::MoveBaseGoal goal;
    // goal.target_pose.header.frame_id = "map"; // these need to change
    // goal.target_pose.header.frame_id = "base_link"; // these need to change
    goal.target_pose.header.frame_id = frame_id; // these need to change
    goal.target_pose.header.stamp = ros::Time();

    goal.target_pose.pose.position.x = target.x;
    goal.target_pose.pose.position.y = target.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = get_quad(target);
    return goal;
}


// set goal position for robot, 
// Input target --> goal to go
void set_goal(std::string frame_id,TargetLocation target){
    
    MoveBaseClient ac("move_base", true);
    // static MoveBaseClient ac("ridgeback/move_base", true);
    // MoveBaseClient ac("move_base_simple", true);

    // //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up!");
    }

    //create goal & send it
    move_base_msgs::MoveBaseGoal goal=create_goal(frame_id,target);
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ROS_INFO_STREAM("Current Goal State -->" << ac.getState().toString().c_str());

}

bool get_Action_state(){
    static MoveBaseClient client("move_base", true);
    ROS_INFO("wtf?");
    // //wait for the action server to come up
    while(!client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up!");
    }

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO_STREAM("Goal SUCCEEDED");
        return true;
    }
    else{
        ROS_INFO_STREAM("Current Goal State -->" << client.getState().toString().c_str());
    }
    return false;
}

void patrol_mode(){
    TargetLocation starting_pos{2,2,0};
    TargetLocation forward{2,0,0};
    TargetLocation rotare{0,0,PI};

    // std::string frame_id  = "base_link";
    // std::string frame_id  = "base_footprint";
    std::string frame_id  = "map";
    static int state = 0;

    if(get_Action_state()){
        state+=1;
    }
    ROS_ERROR_STREAM("patrol state --> "<< state);

    switch(state){
        case 0:
            set_goal(frame_id,starting_pos); // go to starting position
            break;
        case 1:
            set_goal(frame_id,forward); // move forward
            break;
        case 2:
            set_goal(frame_id,rotare); // rotate 180 degrees
            break;
        case 3:
            state = 0;
            break;
        default : 
            ROS_ERROR("Unknow state in patrol function");
    }  
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
    auto frequency = 2;
    ros::Rate loop_rate(frequency);
    TargetLocation home_loc{-0.5,2.7,0};
    TargetLocation kitchen_loc{1.5,0,0};
    TargetLocation shelfs_loc{1.8,2.7,0};
    TargetLocation drawers_loc{-0.5,0,0};

    ROS_INFO("Ready to Recieve Commands!");

    while(ros::ok()){
        ROS_INFO_STREAM("Status --> " << getEnumName(map_position));
        ros::spinOnce();
        // if(rec_command){

            switch(map_position){
                case Position::abandon: // cancel the action do not do this 
                    {
                        actionlib_msgs::GoalID msg;
                        msg.id = "";
                        cancel_pub.publish(msg);
                        map_position = Position::idle;
                    }
                    
                    break;
                case Position::kitchen:
                    set_goal("map",kitchen_loc);
                    map_position = Position::idle;
                    break;
                case Position::shelfs:
                    set_goal("map",shelfs_loc);
                    map_position = Position::idle;
                    break;
                case Position::home:
                    set_goal("map",home_loc);
                    map_position = Position::idle;
                    break;
                case Position::drawers:
                    set_goal("map",drawers_loc);
                    map_position = Position::idle;
                    break;
                case Position::patrol:
                    patrol_mode();
                    break;
                case Position::idle:{
                    break;

                }
                default : 
                    ROS_ERROR("Unknow rec_command");
            }   
            // rec_command = false;
        // }


        geometry_msgs::TransformStamped transformStamped;
        try{
        transformStamped = tfBuffer.lookupTransform("map", "base_link",ros::Time(0));
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



