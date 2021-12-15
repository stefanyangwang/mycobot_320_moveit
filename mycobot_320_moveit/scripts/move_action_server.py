#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander

from geometry_msgs.msg import PoseStamped
from pymycobot.mycobot import MyCobot
import actionlib
from mycobot_320_moveit.msg import *



class MoveItPlanning:
    def __init__(self):
        # initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize a rospy node
        #rospy.init_node("mycobot_moveit")
        self.robot = moveit_commander.RobotCommander()

        # provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()
        # an interface to a planning group (group of joints), it can be used to plan and execute motions
        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        # print the name of the end-effector link for this group
        self.end_effector_link = self.arm.get_end_effector_link()
        print ("============ End effector link: %s" % self.end_effector_link)

        self.reference_frame = "base"
        #self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        

        
        
        



    def MultiMoveServer(self, goal):
        # Initializing action variables
        result = MoveResult()
        
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = self.reference_frame
        approach_pose.header.stamp = rospy.Time.now()
        approach_pose.pose.position.x = goal.targetPosition[0].x
        approach_pose.pose.position.y = goal.targetPosition[0].y
        approach_pose.pose.position.z = goal.targetPosition[0].z
        approach_pose.pose.orientation.x = goal.targetPosition[0].ox
        approach_pose.pose.orientation.y = goal.targetPosition[0].oy
        approach_pose.pose.orientation.z = goal.targetPosition[0].oz
        approach_pose.pose.orientation.w = goal.targetPosition[0].ow
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(approach_pose, self.end_effector_link)
        traj = self.arm.plan()
        if len(traj.joint_trajectory.points) == 0:
            self.multi_move_server.set_aborted(result, "aborted")
        else:
            self.arm.execute(traj)
            self.arm.stop()
            rospy.sleep(2)
            # Formatting the target pose received through the goal into target_pose variable
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = goal.targetPosition[1].x
            target_pose.pose.position.y = goal.targetPosition[1].y
            target_pose.pose.position.z = goal.targetPosition[1].z
            target_pose.pose.orientation.x = goal.targetPosition[1].ox
            target_pose.pose.orientation.y = goal.targetPosition[1].oy
            target_pose.pose.orientation.z = goal.targetPosition[1].oz
            target_pose.pose.orientation.w = goal.targetPosition[1].ow

            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target_pose, self.end_effector_link)
            traj = self.arm.plan()
            self.arm.execute(traj)
            self.arm.stop()
            self.multi_move_server.set_succeeded(result, "succeded")
        

    def MoveHomeServer(self, goal):
        rospy.sleep(2)
        result = MoveResult()
        self.arm.set_named_target("init_pose")
        self.arm.go()
        self.arm.stop()
        self.move_home_server.set_succeeded(result, "succeded")
    
    def start(self):
        self.multi_move_server = actionlib.SimpleActionServer('multi_move', MultiMoveAction, self.MultiMoveServer, False)
        self.move_home_server = actionlib.SimpleActionServer('move_home', MoveHomeAction, self.MoveHomeServer, False)
        self.multi_move_server.start()
        self.move_home_server.start()


if __name__ == "__main__":
    sys.stdout.write('\33]0;move_server node\a')
    sys.stdout.flush()
    rospy.init_node("move_server")
    moveit_server = MoveItPlanning()
    moveit_server.start()
    print('Movement server ready')
    rospy.spin()