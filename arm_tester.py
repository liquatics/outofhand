#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('arm_tester')


# ================ #
# PUBLISHERS      #
# --------------------------------------------------------------------------- #
# Will send info to these channels based on input from evaluator.             #
# =========================================================================== #

arm_pub = rospy.Publisher('/right_arm_controller/command',
                           JointTrajectory, queue_size=10)


# ================ #
# DEFINITIONS      #
# --------------------------------------------------------------------------- #
# Variables used throughout the node.                                         #
# Hand gesture parameters do not need an array and are defined as:            #
# [thumb, pointer, middle, ring, pinky]                                       #
#                                                                             #
# Arm gesture parameters only include shoulder_medial_joint and elbow_joint.  #
# All others in the array should be set as 0.                                 #
# =========================================================================== #

arm_names = ['shoulder_medial_joint', 'elbow_joint']

# 2 is max elbow_joint -2 is min
# 3.2 is 180 degrees

test = [0, -1.6]
default = [0, 0]
max_left = [0, 2]
max_right = [3.2, 2]
middle = [1.6, 1]


arm_gesture2 = JointTrajectory()
arm_gesture2.joint_names = arm_names

arm_gesture_points2 = JointTrajectoryPoint()
arm_gesture_points2.positions = max_right
arm_gesture_points2.time_from_start = rospy.Duration(2)
arm_gesture2.points = [arm_gesture_points2]


# ================== #
# Callback Functions #
# ----------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action. #
# ======================================================================= #


# def setArmGesture(msg):
#     global arm_gesture
#
#     if msg == "lower":
#         arm_gesture = arm_gesture1
#     else:
#         arm_gesture.points = [JointTrajectoryPoint(positions=raised)]
#
#
# arm_gesture = JointTrajectory()
# arm_gesture.joint_names = arm_names

# ================ #
# SUBSCRIBERS      #
# --------------------------------------------------------------------------- #
# Will evaluate info sent on these channels directly from evaluator terminal. #
# =========================================================================== #
#rospy.Subscriber("/arm_gestures", String, setArmGesture)

empty_gesture = Empty()

rate = rospy.Rate(10)
gesture_state = "IDLE"

# Fruit loop
# while not rospy.is_shutdown():
#     time_init = rospy.get_rostime()
    # while rospy.get_rostime() < time_init + rospy.Duration(2):
rospy.sleep(1)
arm_pub.publish(arm_gesture2)
rospy.sleep(0.1)

rospy.spin()
