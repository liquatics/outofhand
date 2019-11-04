#!/usr/bin/env python
import rospy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

# initialize the node for hand movement
rospy.init_node('hand_mover')

# subscribe to /hand_command topic for incoming evaluator commands
rospy.Subscriber("/hand_command", String, gesture_callback)

# publish the /hand_controller/command for actions
hand_pub = rospy.Publisher('/hand_controller/command',
                           JointTrajectory, queue_size=10)

# arrays for the different fingers on the hand
hand_array = []

# *****************************************************************************
#                               setup for Odds-Evens
# *****************************************************************************

# initialize odds and assign hand_array
odds = JointTrajectory()
odds.joint_names = hand_array
# set points for odds finger positions, only has pointer finger extend
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
odds_points = JointTrajectoryPoint()
odds_points.positions = []
odds_points.velocities = []*
# send points array back to odds for loop commands
odds.points = [odds_point]


# initialize evens and assign hand_array
evens = JointTrajectory()
evens.joint_names = hand_array
# set points for evens finger positions, only has pointer/middle finger extend
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
evens_points = JointTrajectoryPoint()
evens_points.positions = []
evens_points.velocities = []*
# send points array back to evens for loop commands
evens.points = [evens_points]
