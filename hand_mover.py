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
head_pub = rospy.Publisher('/hand_controller/command',
                           JointTrajectory, queue_size=10)

# arrays for the different fingers on the hand
hand_array = []

# ******************************************************************
#                       setup for Odds-Evens
# ******************************************************************
odds = JointTrajectory()
odds.joint_names = hand_array
