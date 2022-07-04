#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


rospy.init_node('buzz_test')
# pub = rospy.Publisher('/senseglove/0/rh/controller/trajectory/command', JointTrajectory, queue_size=1)
client = actionlib.SimpleActionClient('/senseglove/0/rh/controller/trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
goal = FollowJointTrajectoryGoal()
# # goal.
# msg = JointTrajectory()
# msg.joint_names = ['thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
# point = JointTrajectoryPoint()
# point.positions = [20.0, 20.0, 20.0, 20.0, 20.0]
# point.velocities = [20.0, 20.0, 20.0, 20.0, 20.0]
# point.effort = [20.0, 20.0, 20.0, 20.0, 20.0]
# point.time_from_start = 5.0
# msg.points.append(point)
# hap_cmd = JointTrajectory()
# hap_cmd.header = Header()
# hap_cmd.header.stamp = rospy.Time.now()
# hap_cmd.joint_names = 

point = JointTrajectoryPoint()
break_val = 0
vib_val = 0
point.positions = [break_val, break_val, break_val, break_val, break_val, vib_val, vib_val, vib_val, vib_val, vib_val]  # what you will!
point.time_from_start = rospy.Duration.from_sec(0.1)
goal.trajectory.joint_names = ['thumb_brake', 'index_brake', 'middle_brake', 'ring_brake', 'pinky_brake', 
                               'thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
goal.trajectory.points.append(point)
rate = rospy.Rate(1)
while rospy.is_shutdown() is False:
    goal.trajectory.header.stamp = rospy.Time.now()
    # goal.trajectory.header.stamp.nsecs -= 25000
    client.send_goal(goal)
    client.wait_for_result()
    rate.sleep()