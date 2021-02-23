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
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>


#define PI 3.14159265359
enum Position {kitchen, shelfs, home, drawers, charging, forward, rotate };
enum State {abandon, patrol,idle,goTO };
enum State_Patrol {forward_patrol, rotate_patrol};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



struct TargetLocation{
    double x = 0;
    double y = 0;
    double orientation = 0;

};


class ROBOT_STATE
{
    public:
        
        std::string get_position_name(){
            switch(target){
                    case Position::kitchen:
                        return "kitchen";
                        break;
                    case Position::shelfs:
                        return "shelfs";
                        break;
                    case Position::home:
                        return "home";
                        break;
                    case Position::drawers:
                        return "drawers";
                        break;
                    case Position::charging:
                        return "charging";
                        break;
                    case Position::forward:
                        return "forward";
                        break;
                    case Position::rotate:
                        return "rotate";
                        break;
                    default : 
                        ROS_ERROR("get_position_name --> Unknow value");
                        return "home";

            }              
        }
        
        TargetLocation get_position(){
            switch(target){
                    case Position::kitchen:
                        return kitchen_goal;
                        break;
                    case Position::shelfs:
                        return shelfs_goal;
                        break;
                    case Position::home:
                        return home_goal;
                        break;
                    case Position::drawers:
                        return drawers_goal;
                        break;
                    case Position::charging:
                        return charging_goal;
                        break;
                    case Position::forward:
                        return forward_goal;
                        break;
                    case Position::rotate:
                        return rotate_goal;
                        break;
                    default : 
                        ROS_ERROR("get_position_name --> Unknow value");
                        return home_goal;

            }  
        }
       
        void set_position(std::string place){
            if(place=="kitchen")       target = Position::kitchen;
            else if(place=="shelfs")   target = Position::shelfs;
            else if(place=="home")     target = Position::home;
            else if(place=="drawers")  target = Position::drawers;
            else if(place=="charging") target = Position::charging;
            else if(place=="forward") target = Position::forward;
            else if(place=="rotate") target = Position::rotate;
            else{
                ROS_ERROR("set_position --> Unknow value");
                state = State::idle;
            }
            

        }

        std::string get_state_name(){
            switch(state){
                    case State::abandon:
                        return "abandon";
                        break;
                    case State::patrol:
                        return "patrol";
                        break;
                    case State::idle:
                        return "idle";
                        break;
                    case State::goTO:
                        return "goTO --> " + get_position_name();
                        break;
                    default : 
                        ROS_ERROR("get_state_name --> Unknow Command");
                        return "idle";

            }   
        }

        void set_state(State stateTemp){
            state = stateTemp;
        }

        State get_state(){
            return state;
        }
    
    
    private:
        
    Position target=Position::home;
    State state=State::idle; 
    TargetLocation kitchen_goal{-0.5,2.7,0};
    TargetLocation shelfs_goal{1.5,0,0};
    TargetLocation home_goal{1.8,2.7,0};
    TargetLocation drawers_goal{-0.5,0,0};
    TargetLocation charging_goal{0,0,0};
    TargetLocation forward_goal{2,0,0};
    TargetLocation rotate_goal{0,0,PI};
};




class MyNode
{
    public:
        MyNode():
            nh{},
            serviceCommader(nh.advertiseService("commander", &MyNode::command_callback,this)),
            actionClient("move_base", true),
            timer(nh.createTimer(ros::Duration(1), &MyNode::main_loop, this))
         {
            

            //wait for the action server to come up
            while(!actionClient.waitForServer(ros::Duration(2.0))){
                ROS_INFO("Waiting for the move_base action server to come up!");
            }
            ROS_INFO("Ready to Recieve Commands!");

            update_robot_position();
         }

        // implementation of service for controling the robot
        bool command_callback(commander::commandSRV::Request  &req,
                              commander::commandSRV::Response &res){
            command_recieved = true;
                switch(req.input_command){
                    case 0:
                        robot_state.set_state(State::abandon);
                        res.result = "Recieved Command --> abandon";
                        break;
                    case 3:
                        robot_state.set_state(State::patrol);
                        res.result = "Recieved Command --> patrol";
                        break;
                    case 2:
                        robot_state.set_state(State::idle);
                        res.result = "Recieved Command --> idle";
                        break;
                    case 1:
                        robot_state.set_state(State::goTO);
                        robot_state.set_position(req.place);
                        res.result = "Recieved Command --> goTO";
                        break;
                    default : 
                        ROS_ERROR("service command --> Unknow Command");
                        res.result = "Unknow Command";
                        command_recieved = false;
                        return false;

                }   
                ROS_INFO_STREAM(res.result);
                return true;
            }

        
        // get yaw from Quaternion
        double get_yaw(geometry_msgs::Quaternion q){
            static double roll, pitch, yaw;
            tf::Quaternion tfq;
            tf::quaternionMsgToTF(q, tfq);
            tf::Matrix3x3(tfq).getEulerYPR(yaw,pitch,roll);
            return yaw;
        }

        // convert yaw to Quaternion
        geometry_msgs::Quaternion get_quad(TargetLocation target){
            tf2::Quaternion q;
            q.setRPY(0, 0, target.orientation);
            q.normalize();
            geometry_msgs::Quaternion quad_msg;
            quad_msg = tf2::toMsg(q);
            return quad_msg;
        }

        // create goal message
        move_base_msgs::MoveBaseGoal create_goalMSG(std::string frame_id,TargetLocation target){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = frame_id;
            goal.target_pose.header.stamp = ros::Time();
            goal.target_pose.pose.position.x = target.x;
            goal.target_pose.pose.position.y = target.y;
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation = get_quad(target);
            return goal;
        }


        // updates current position of the robot
        void update_robot_position(){
            static tf2_ros::Buffer tfBuffer;
            static tf2_ros::TransformListener tfListener(tfBuffer);
            static geometry_msgs::TransformStamped transformStamped;

            try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link",ros::Time(0));
            robot_pos = {transformStamped.transform.translation.x,
                         transformStamped.transform.translation.y,
                 get_yaw(transformStamped.transform.rotation)};

            }
            catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            }
        }

        // return true if the goal is reached
        bool get_Action_state(){
            auto status = actionClient.getState();
            if (status == actionlib::SimpleClientGoalState::SUCCEEDED || status == actionlib::SimpleClientGoalState::LOST ){
                ROS_INFO_STREAM("Goal SUCCEEDED");
                return true;
            }
            else{
                ROS_INFO_STREAM("Current Goal State -->" << actionClient.getState().toString().c_str());
            }
            return false;
        }

        // send goal position for robot, 
        void send_goal(std::string frame_id,TargetLocation target){
            //create goal & send it
            move_base_msgs::MoveBaseGoal goal=create_goalMSG(frame_id,target);
            ROS_INFO("Sending goal");
            actionClient.sendGoal(goal);
        }

        void goTO_state(std::string frame_id){
            TargetLocation target_pos = robot_state.get_position();
            send_goal(frame_id,target_pos);
        }

        void patrol_shelfs(){
            TargetLocation starting_pos{2,2,0};

            // std::string frame_id  = "base_link";
            // std::string frame_id  = "base_footprint";
            static std::string frame_id  = "base_link";
            static State_Patrol patrol_state = State_Patrol::forward_patrol;

            if(get_Action_state()){

                switch(patrol_state){
                    case State_Patrol::forward_patrol:
                        ROS_ERROR_STREAM("patrol state --> forward_patrol ");
                        robot_state.set_position("forward");
                        // robot_state.set_position("home");
                        goTO_state(frame_id); // go to starting position
                        patrol_state = State_Patrol::rotate_patrol;
                        break;
                    case State_Patrol::rotate_patrol:
                        ROS_ERROR_STREAM("patrol state --> rotate_patrol ");
                        robot_state.set_position("rotate");
                        // robot_state.set_position("kitchen");
                        goTO_state(frame_id); // move forward
                        patrol_state = State_Patrol::forward_patrol;
                        break;
                    default : 
                        ROS_ERROR("Unknow state in patrol function");

                }
            }
        }

        void cancel_goal(){
            actionClient.cancelAllGoals();
        }

        double rad2Degree(double radians){
            return radians*180/PI;
        }

        //main loop
        void main_loop(const ros::TimerEvent &)
        {
            //print current state
            ROS_INFO_STREAM("Current state --> " << robot_state.get_state_name());
            ROS_INFO_STREAM("Robot position --> " << robot_pos.x << " " << robot_pos.y << " " << rad2Degree(robot_pos.orientation));
            get_Action_state();
            update_robot_position();

            switch(robot_state.get_state()){
                    case State::abandon:
                        if(command_recieved){
                            cancel_goal();
                            command_recieved = false;
                        }
                        break;
                    case State::patrol:
                        patrol_shelfs();
                        break;
                    case State::idle:
                        break;
                    case State::goTO:
                        if(command_recieved){
                            goTO_state("map");
                            command_recieved = false;
                        }
                        break;
                    default : 
                        ROS_ERROR("main loon --> Unknow state...?");

            }  


        }

    private:
        ros::NodeHandle nh;
        ros::ServiceServer serviceCommader;
        ros::Timer timer;
        bool command_recieved=false;
        ROBOT_STATE robot_state;
        MoveBaseClient actionClient;
        TargetLocation robot_pos{0,0,0};
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    MyNode node;
    ros::spin();
    return 0;
}
