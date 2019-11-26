#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# LET'S GOOOOOOOO
rospy.init_node('arm_tester')


# ================ #
# PUBLISHERS       #
# --------------------------------------------------------------------------- #
# Will send info to these channels based on input from evaluator.             #
# =========================================================================== #

arm_pub = rospy.Publisher('/right_arm_controller/command',
                           JointTrajectory, queue_size=10)


# ================ #
# DEFINITIONS      #
# --------------------------------------------------------------------------- #
# Arm gesture parameters only include shoulder_medial_joint and elbow_joint.  #
# All others in the array should be set empty.                                #
# 2 is the max value for elbow_joint, while -2 is the min                     #
# 3.2 indicates a 180 degree rotation for shoulder_medial_joint               #
# =========================================================================== #

arm_names = ['shoulder_medial_joint', 'elbow_joint']

# Blank gesture that is set depending on the desired position in the main loop
arm_gesture = JointTrajectory()
arm_gesture.joint_names = arm_names
arm_gesture_points = JointTrajectoryPoint()
arm_gesture_points.positions = []
arm_gesture_points.time_from_start = rospy.Duration(3)
arm_gesture.points = [arm_gesture_points]

position = String()

# point locations
test = [0, -1.6]
default = [0, 0]
max_left = [0, 2]
max_right = [3.2, 2]
middle = [1.6, 1]


# ================== #
# FUNCTIONS          #
# --------------------------------------------------------------------------- #
# Evaluates an action and sets the gesture points to reflect that action.     #
# =========================================================================== #

def setArmGesture(msg):
    global position
    global arm_gesture
    position.data = msg.data


# ================ #
# SUBSCRIBERS      #
# --------------------------------------------------------------------------- #
# Will evaluate info sent on these channels directly from evaluator terminal. #
# =========================================================================== #

rospy.Subscriber("/arm_gestures", String, setArmGesture)


# ================ #
# SETUP            #
# --------------------------------------------------------------------------- #
# Helps establish setup for the main loop.                                    #
# =========================================================================== #

rate = rospy.Rate(10)

# Fruit loop
while not rospy.is_shutdown():
    if position.data == "test":
        arm_gesture_points.positions = test
    elif position.data == "default":
        arm_gesture_points.positions = default
    elif position.data == "max_left":
        arm_gesture_points.positions = max_left
    elif position.data == "max_right":
        arm_gesture_points.positions = max_right
    elif position.data == "middle":
        arm_gesture_points.positions = middle

    rospy.sleep(1)
    arm_pub.publish(arm_gesture)
    rospy.sleep(0.1) # more of a catnap
