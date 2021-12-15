#!/usr/bin/env python

import rospy
import actionlib
from mycobot_320_moveit.msg import *

def move_client():
    client = actionlib.SimpleActionClient('move', MultiMoveAction)
    print("Waiting for the move server ...")
    client.wait_for_server()
    print("\n --- Server ready --- \n")
    goal = MultiMoveGoal()
    approach_goal = robot_goals()
    approach_goal.x = 0.25409088624289733
    approach_goal.y = -0.03248359876201828
    approach_goal.z = 0.11967745058037846
    approach_goal.ox = 0.3852122476586819
    approach_goal.oy = 0.5951954753491578
    approach_goal.oz = -0.3842176257717917
    approach_goal.ow =  0.5913803229935233
    goal.targetPosition.append(approach_goal)
    target_goal = robot_goals()
    target_goal.x = 0.2539028024599151
    target_goal.y = -0.0322778144393383
    target_goal.z = 0.06967822781338817
    target_goal.ox = 0.3852122476586819
    target_goal.oy = 0.5951954753491578
    target_goal.oz = -0.3842176257717917
    target_goal.ow = 0.5913803229935233
    goal.targetPosition.append(target_goal)
    client.send_goal(goal)
    client_state = client.get_state()
    while client_state != 3: # not in [2,3,4,5,8]
        client_state = client.get_state()
        # ABORTED : 4
        if client_state == 4:
            return 'target_not_reached'
    print(client_state)
    print('--- Movement completed ---')
    return 'target_reached'

if __name__ == '__main__':
         rospy.init_node('move_client')
         result = move_client()
         print(result)