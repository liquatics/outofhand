#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('hand_tester')


# ================ #
# PUBLISHERS      #
# --------------------------------------------------------------------------- #
# Will send info to these channels based on input from evaluator.             #
# =========================================================================== #
hand_pub = rospy.Publisher('/hand_controller/command',
                           JointTrajectory, queue_size=10)
test_pub = rospy.Publisher('/hand_controller/command',
                           String, queue_size=10)


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
pointer = [1, 0, 1, 1, 1]
peace = [1, 0, 0, 1, 1]
thumbs_up = [0, 1, 1, 1, 1]
fist = [1, 1, 1, 1, 1]
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


gesture_state = String()
hand_gesture = JointTrajectory()

# ================== #
# Callback Functions #
# ----------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action. #
# ======================================================================= #
def setHandGesture(msg):
    global hand_gesture
    global gesture_state

    gesture_state = "GESTURING"
    test_pub.publish("received")

    if msg == 'pointer':
        hand_gesture.points = hand_gesture1.points

    elif msg == "peace":
        hand_gesture.points = hand_gesture2.points
    elif msg == "thumbs_up":
        hand_gesture.points = hand_gesture3.points
    elif msg == "fist":
        hand_gesture.points = hand_gesture4.points
    else:
        hand_gesture.points = hand_gesture5.points


# ================ #
# SUBSCRIBERS      #
# --------------------------------------------------------------------------- #
# Will evaluate info sent on these channels directly from evaluator terminal. #
# =========================================================================== #
rospy.Subscriber("/hand_utility_gestures", String, setHandGesture)


rate = rospy.Rate(10)

# Fruit loop
while not rospy.is_shutdown():

    if(gesture_state == "GESTURING"):
        hand_pub.publish(hand_gesture)
        rospy.sleep(0.1)
