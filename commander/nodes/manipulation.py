#!/usr/bin/env python3
from moveit_commander import roscpp_initialize
import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion
import time

#saved states 
snake = [-1.5797138671875, -1.63575, -1.6687451171875, -0.175255859375, -0.635630859375, -1.8552236328125, -4.6607734375]
snake2 = [0.1599619140625, -0.698787109375, -0.2838037109375, 0.994494140625, 0.2812392578125, 0.87126953125, -1.6042626953125]
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
        # self.right_gripper = Gripper('hand')
        # self.right_gripper.calibrate(timeout=1.0)

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


    def get_position(self):
        pos = self.group.get_current_pose("right_hand")
        rospy.loginfo(f"ARM Position --> {pos}")



def main():
    """ The main() function. """
    rospy.loginfo(sys.argv)
    rospy.init_node('arm',log_level=rospy.DEBUG)
    handler = Manipulator()


    while(not rospy.is_shutdown()):
        #handler.get_position()
        handler.go_to_state(snake2)
        handler.go_to_state(snake)
        rospy.sleep(1)
    


    
    #while(not rospy.is_shutdown()):
        #handler.get_position()
        #rospy.sleep(1)
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass