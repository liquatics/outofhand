#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('basic_hand')

# ================ #
# SUBSCRIBERS      #
# ------------------------------------------ #
# Will evaluate info sent on these channels. #
# ========================================== #

rospy.Subscriber('/hand_utility_gestures', String, setHandGesture)
rospy.Subscriber('/arm_utility_gestures', String, setArmGesture)

# ================ #
# Publishers      #
# ----------------------------------------------------------------------- #
# Will send info to these channels.                                       #
# ======================================================================= #

hand_pub = rospy.Publisher('/hand_controller/command',
                           JointTrajectory, queue_size=10)
arm_pub = rospy.Publisher('/right_arm/request',
                           JointTrajectory, queue_size=10)
arm_watchdog = rospy.Publisher('/right_arm/watchdog',
                                Empty, queue_size=10)

# ================ #
# Definitions      #
# ----------------------------------------------------------------------- #
# Variables used throughout the node.                                     #
# Hand gesture parameters are defined as:                                 #
#                                                                         #
# [thumb, pointer, middle, ring, pinky]                                   #
# ======================================================================= #

hand_gesture = JointTrajectory()
hand_gesture.points = [JointTrajectoryPoint(positions=open_hand)]
fist = [1, 1, 1, 1, 1]
open_hand = [0, 0, 0, 0, 0]

arm_gesture = JointTrajectory()
arm_names = ['shoulder_flexation_joint',
             'shoulder_abduction_joint',
             'shoulder_medial_joint',
             'elbow_joint',
             'wrist_joint',
             ]
raised = [0, 0, 0, 0, 0]
lowered = [1, 1, 1, 1, 1]


# =================== #
# setHandGesture(msg) #
# ----------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action. #
# ======================================================================= #

def setHandGesture(msg):
    global hand_gesture

    if msg == "fist":
        hand_gesture.points = [JointTrajectoryPoint(positions=fist)]
    else:
        hand_gesture.points = [JointTrajectoryPoint(positions=open_hand)]


# =================== #
# setArmGesture(msg)  #
# ----------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action. #
# ======================================================================= #

def setArmGesture(msg):
    global arm_gesture

    if msg == "lower":
        arm_gesture.points = [JointTrajectoryPoint(positions=lowered)]
    else:
        arm_gesture.points = [JointTrajectoryPoint(positions=raised)]

# Fruit loop
while not rospy.is_shutdown():
    hand_pub.publish(hand_gesture)
    arm_pub.publish(arm_gesture)
    rospy.sleep(0.1)
