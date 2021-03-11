#!/usr/bin/env python3
from moveit_commander import roscpp_initialize
import moveit_commander
import actionlib 
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf

#saved states 
snake = [-1.5797138671875, -1.63575, -1.6687451171875, -0.175255859375, -0.635630859375, -1.8552236328125, -4.6607734375]
snake2 = [0.1599619140625, -0.698787109375, -0.2838037109375, 0.994494140625, 0.2812392578125, 0.87126953125, -1.6042626953125]

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

        self.client = actionlib.SimpleActionClient('/robot/end_effector/right/gripper_action/', GripperCommandAction)

        # self.right_gripper = Gripper('hand')
        # self.right_gripper.calibrate(timeout=1.0)


        #tf listener
        self.listener = tf.TransformListener()
    
    def open_gripper(self):
        goal = GripperCommandGoal()
        goal.command.position= 0.2
        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self):
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


    def check_tf(self):
        # print(sel f.group.get_current_pose())
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/tag_0', rospy.Time(0))
            print (trans)
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
        
        is_tf = self.check_tf()
        if(is_tf):
            self.go_to_state(snake2)
            self.grab_object([0.1,0.0,0.1])
            self.grab_object([0.1,0.0,-0.02])
            # close gripper
            self.go_to_state(snake2)

def main():
    """ The main() function. """
    rospy.loginfo(sys.argv)
    rospy.init_node('arm',log_level=rospy.DEBUG)
    handler = Manipulator()

    handler.go_to_state(snake2)
    while(not rospy.is_shutdown()):
        #handler.get_position()
        # handler.go_to_state(snake2)
        # handler.go_to_state(snake)
        is_tf = handler.check_tf()
        print(is_tf)
        handler.toggle_gripper()
        # handler.get_offset()
        if(is_tf):
            #handler.grab_object([0.1,0.0,0.1])
            #handler.grab_object([0.1,0.0,-0.02])
           # handler.toggle_gripper()
            exit()


        
            


        rospy.sleep(1)
    


    
    #while(not rospy.is_shutdown()):
        #handler.get_position()
        #rospy.sleep(1)
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass