#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##
import math
import time
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectoryPoint
import sys
import copy
import threading
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


joyState = Joy()

def cb_joy(msg):
    global joyState
    joyState = msg

if __name__ == '__main__':
    global joyState#, jntState, trajectory

    rospy.init_node('joy2cmd', anonymous=True)

    sub_joy = rospy.Subscriber("/joy", Joy, cb_joy)
    #sub_jointState = rospy.Subscriber("/joint_states", JointState, cb_jntState)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator_i5"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
							moveit_msgs.msg.DisplayTrajectory, 								queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    print('start')
 

    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
    	joint_goal = group.get_current_joint_values()
        
        if len(joyState.axes) is 8:
		if int(joyState.buttons[0]) == 1:

                	print("aubo_robot.go_to_joint_state()")
                
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go([0,0,0,0,0,0], wait=True)
                	group.stop()
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			
            	elif int(joyState.axes[7]) == -1:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[0] = joint_goal[0]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
	    	elif int(joyState.axes[7]) == 1:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[0] = joint_goal[0]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
	    	elif int(joyState.axes[6]) == -1:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[1] = joint_goal[1]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
	    	elif int(joyState.axes[6]) == 1:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[1] = joint_goal[1]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[0]) > 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[2] = joint_goal[2]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[0]) < 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[2] = joint_goal[2]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[1]) > 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[3] = joint_goal[3]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[1]) < 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[3] = joint_goal[3]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[3]) > 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[4] = joint_goal[4]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")
            	elif float(joyState.axes[3]) < 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[4] = joint_goal[4]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")		
            	elif float(joyState.axes[4]) > 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[5] = joint_goal[5]+float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")	
            	elif float(joyState.axes[4]) < 0:
			print(joyState.axes)
			print("current_state_is_",joint_goal)
                	print("aubo_robot.go_to_joint_state()")
			joint_goal = group.get_current_joint_values()
			joint_goal[5] = joint_goal[5]-float(0.1)
                	currentState = robot.get_current_state()
                	position = currentState.joint_state.position
                	group.go(joint_goal, wait=True) 
                	current_joints = group.get_current_joint_values()
			all_close(currentState, current_joints, 0.01)
			print("next")	
    sub_joy.unregister()
#    pub_cmd.unregister()


