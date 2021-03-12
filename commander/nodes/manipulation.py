#!/usr/bin/env python3
from moveit_commander import roscpp_initialize
import moveit_commander
import actionlib 
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion, Twist
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import *
import tf
import math 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

#saved states
snake  = [-0.0579208984375, -1.9296689453125, 0.0431044921875, 1.2144580078125, -0.1240341796875, 1.360943359375, -1.3926005859375]
 
#snake = [-1.5797138671875, -1.63575, -1.6687451171875, -0.175255859375, -0.635630859375, -1.8552236328125, -4.6607734375]
snake2 = [0.1599619140625, -0.698787109375, -0.2838037109375, 0.994494140625, 0.2812392578125, 0.87126953125, -1.6042626953125]
place_table_pos = [-1.4636396484375, -0.928876953125, -0.3344345703125, 0.3650927734375, 0.142884765625, 1.805130859375, -1.394240234375]

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class Manipulator:
    """ Helper class for node arm_control.
    Will sort cups then build tower by grabbing and placing cups using either one hand or both hands.
    """
    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        #moveit_commander.roscpp_initialize(joint_state_topic)
        self.robot = moveit_commander.RobotCommander(robot_description='robot_description', ns="robot")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm", robot_description='robot_description')

        self.client = actionlib.SimpleActionClient('/robot/end_effector/right/gripper_action', GripperCommandAction)
        self.move_base_action = actionlib.SimpleActionClient('/ridgeback/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server()
        self.forward = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.hunter_tag = "/tag_0"
        # self.right_gripper = Gripper('hand')
        # self.right_gripper.calibrate(timeout=1.0)


        #tf listener
        self.listener = tf.TransformListener()

    def rotate_180(self):
        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        q_rot = quaternion_from_euler(0, 0,math.pi)
        q_rot = quaternion_multiply(q_rot, rot)
        print(q_rot)
        print(type(q_rot))
        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = trans[0]
        goal.target_pose.pose.position.y = trans[1]
        goal.target_pose.pose.position.x = trans[2]
        goal.target_pose.pose.orientation.x = q_rot[0]
        goal.target_pose.pose.orientation.y = q_rot[1]
        goal.target_pose.pose.orientation.z = q_rot[2]
        goal.target_pose.pose.orientation.w = q_rot[3]

        self.move_base_action.send_goal(goal)
        wait = self.move_base_action.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_action.get_result()
    
    def open_gripper(self):
        print ("Opening Gripper ...")
        goal = GripperCommandGoal()
        goal.command.position= 0.2
        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self):
        print ("Closing Gripper ...")

        goal = GripperCommandGoal()
        goal.command.position= 0
        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def go_to_state(self,joint_goal):
        self.group.set_joint_value_target(joint_goal)
        (result, plan, frac, errCode) = self.group.plan()
        rospy.loginfo(f"err code = {errCode}")
        result = self.group.execute(plan, wait=True)

    def go_to_pose(self,pose_goal):
        print ("Going to :" + str(pose_goal))
        self.group.set_pose_target(pose_goal)
        (result, plan, frac, errCode) = self.group.plan()
        rospy.loginfo(f"err code = {errCode}")
        result = self.group.execute(plan, wait=True)
        # self.group.stop()
        # self.group.clear_pose_target(end_effector_link = "hand")


    def get_offset(self):
        print(self.group.get_current_pose().pose)
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/tag_0', rospy.Time(0))
            print(trans)
            print("diff")
            pose = self.group.get_current_pose()
            eef_pose = [pose.pose.position.x-trans[0],pose.pose.position.y-trans[1],pose.pose.position.z-trans[2]]
            print(eef_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("no tf found")




    def get_position(self):
        pos = self.group.get_current_pose("right_hand")
        rospy.loginfo(f"ARM Position --> {pos}")

    def get_dist(self,pos):
        return math.sqrt(pos[0]*pos[0] + pos[1]*pos[1])

    def check_tf(self):
        # print(sel f.group.get_current_pose())
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', self.hunter_tag, rospy.Time(0))
            print (trans)
            dist = self.get_dist(trans)
            print("Found tag!")
            print(dist)
            if(dist<1.0):
                return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        return False


    def grab_object(self, off):
        (trans,rot) = self.listener.lookupTransform('/base_link', '/tag_0', rospy.Time(0))
        print(trans)
        trans[0] = trans[0] + off[0]
        trans[1] = trans[1] + off[1]
        trans[2] = trans[2] + off[2]
        quad = [1,0,0,0]
        pose = trans+quad
        self.go_to_pose(pose)

    def pick(self):
        print("Pick object")
        is_tf = self.check_tf()
        if(is_tf):
            self.open_gripper()
            self.go_to_state(snake)
            self.grab_object([0.1,0.0,0.1])
            self.grab_object([0.1,0.0,-0.02])
            self.close_gripper()
            self.go_to_state(snake)

    def place_table(self):
        print("Place object Table")
        self.go_to_state(place_table_pos)
        self.open_gripper()
        self.go_to_state(snake)


    def go_forward(self):
        forward_msg = Twist()
        forward_msg.linear.x = 0.1
        self.forward.publish(forward_msg)

def main():
    """ The main() function. """
    rospy.loginfo(sys.argv)
    rospy.init_node('arm',log_level=rospy.DEBUG)
    handler = Manipulator()
    # print (handler.group.get_current_joint_values())
    # exit()



    handler.go_to_state(snake)
    while(not rospy.is_shutdown()):
        
        is_tf = handler.check_tf()
        print(is_tf)
        handler.go_forward()
        
        if(is_tf):
            print("Pick item")
            handler.pick()
            handler.rotate_180()
            handler.hunter_tag = "/tag_1"
            exit()

        rospy.sleep(0.05)        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass