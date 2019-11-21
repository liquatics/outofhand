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

raised = [1, 1]
lowered = [1.6, 1.6]

arm_gesture1 = JointTrajectory()
arm_gesture1.joint_names = arm_names
arm_gesture_points1 = JointTrajectoryPoint()
arm_gesture_points1.positions = [raised]
arm_gesture1.points = [arm_gesture_points1]

arm_gesture2 = JointTrajectory()
arm_gesture2.joint_names = arm_names
arm_gesture_points2 = JointTrajectoryPoint()
arm_gesture_points2.positions = [lowered]
arm_gesture2.points = [arm_gesture_points2]


# ================== #
# Callback Functions #
# ----------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action. #
# ======================================================================= #


def setArmGesture(msg):
    global arm_gesture

    if msg == "lower":
        arm_gesture = arm_gesture1
    else:
        arm_gesture.points = [JointTrajectoryPoint(positions=raised)]


arm_gesture = JointTrajectory()
arm_gesture.joint_names = arm_names

# ================ #
# SUBSCRIBERS      #
# --------------------------------------------------------------------------- #
# Will evaluate info sent on these channels directly from evaluator terminal. #
# =========================================================================== #
rospy.Subscriber("/arm_gestures", String, setArmGesture)

empty_gesture = Empty()

rate = rospy.Rate(10)
gesture_state = "IDLE"

# Fruit loop
while not rospy.is_shutdown():

    arm_pub.publish(arm_gesture)
    rospy.sleep(0.1)
