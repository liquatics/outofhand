#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('basic_hand')


# ================ #
# PUBLISHERS      #
# --------------------------------------------------------------------------- #
# Will send info to these channels based on input from evaluator.             #
# =========================================================================== #
hand_pub = rospy.Publisher('/hand_controller/command',
                           JointTrajectory, queue_size=10)
arm_pub = rospy.Publisher('/right_arm/request',
                           JointTrajectory, queue_size=10)
arm_watchdog = rospy.Publisher('/right_arm/watchdog',
                                Empty, queue_size=10)


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
pointer = [255, 0, 255, 255, 255]
peace = [255, 0, 0, 255, 255]
thumbs_up = [0, 255, 255, 255, 255]
fist = [255, 255, 255, 255, 255]
open_hand = [0, 0, 0, 0, 0]

hand_gesture1 = JointTrajectory()
hand_gesture1.points = [JointTrajectoryPoint(positions=pointer)]
hand_gesture2 = JointTrajectory()
hand_gesture2.points = [JointTrajectoryPoint(positions=peace)]
hand_gesture3 = JointTrajectory()
hand_gesture3.points = [JointTrajectoryPoint(positions=thumbs_up)]
hand_gesture4 = JointTrajectory()
hand_gesture4.points = [JointTrajectoryPoint(positions=fist)]
hand_gesture5 = JointTrajectory()
hand_gesture5.points = [JointTrajectoryPoint(positions=open_hand)]


arm_names = ['shoulder_flexion_joint',
             'shoulder_abduction_joint',
             'shoulder_medial_joint',
             'elbow_joint',
             'wrist_joint']

raised = [0, 0, 1, 1, 0]
lowered = [0, 0, 0, 0, 0]

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
def setHandGesture(msg):
    global hand_gesture

    if msg == "pointer":
        hand_gesture.points = hand_gesture1.points
    elif msg == "peace":
        hand_gesture.points = hand_gesture2.points
    elif msg == "thumbs_up"
        hand_gesture.points = hand_gesture3.points
    elif msg == "fist"
        hand_gesture.points = hand_gesture4.points
    else:
        hand_gesture.points = hand_gesture5.points
    gesture_state = "GESTURING"

    return gesture_state


def setArmGesture(msg):
    global arm_gesture

    if msg == "lower":
        arm_gesture.points = [JointTrajectoryPoint(positions=lowered)]
    else:
        arm_gesture.points = [JointTrajectoryPoint(positions=raised)]


# ================ #
# SUBSCRIBERS      #
# --------------------------------------------------------------------------- #
# Will evaluate info sent on these channels directly from evaluator terminal. #
# =========================================================================== #
rospy.Subscriber('/hand_utility_gestures', String, setHandGesture)
rospy.Subscriber('/arm_utility_gestures', String, setArmGesture)



rate = rospy.Rate(10)
gesture_state = "IDLE"

# Fruit loop
while not rospy.is_shutdown():

    if(gesture_state == "IDLE"):
        arm_watchdog.publish(Empty)
    elif(gesture_state == "GESTURING")
        hand_pub.publish(hand_gesture)
        arm_pub.publish(arm_gesture)
        rospy.sleep(0.1)
        gesture_state = "IDLE"
