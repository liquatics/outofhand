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
# Arm gesture parameters only include shoulder_medial_joint and elbow_joint.  #
# All others in the array should be set empty.                                #
# 2 is the max value for elbow_joint, while -2 is the min                     #
# 3.2 indicates a 180 degree rotation for shoulder_medial_joint               #
# =========================================================================== #


arm_names = ['shoulder_medial_joint', 'elbow_joint']

arm_gesture = JointTrajectory()
arm_gesture.joint_names = arm_names
arm_gesture_points = JointTrajectoryPoint()
arm_gesture_points.positions = []
arm_gesture_points.time_from_start = rospy.Duration(3)
arm_gesture.points = [arm_gesture_points]

position = String()

test = [0, -1.6]
default = [0, 0]
max_left = [0, 2]
max_right = [3.2, 2]
middle = [1.6, 1]


gest_test = JointTrajectory()
gest_test.joint_names = arm_names
gest_test_points = JointTrajectoryPoint()
gest_test_points.positions = test
gest_test_points.time_from_start = rospy.Duration(3)
gest_test.points = [gest_test_points]

gest_def = JointTrajectory()
gest_def.joint_names = arm_names
gest_def_points = JointTrajectoryPoint()
gest_def_points.positions = default
gest_def_points.time_from_start = rospy.Duration(3)
gest_def.points = [gest_def_points]

gest_maxl = JointTrajectory()
gest_maxl.joint_names = arm_names
gest_maxl_points = JointTrajectoryPoint()
gest_maxl_points.positions = max_left
gest_maxl_points.time_from_start = rospy.Duration(3)
gest_maxl.points = [gest_maxl_points]

gest_maxr = JointTrajectory()
gest_maxr.joint_names = arm_names
gest_maxr_points = JointTrajectoryPoint()
gest_maxr_points.positions = max_right
gest_maxr_points.time_from_start = rospy.Duration(3)
gest_maxr.points = [gest_maxr_points]

gest_mid = JointTrajectory()
gest_mid.joint_names = arm_names
gest_mid_points = JointTrajectoryPoint()
gest_mid_points.positions = middle
gest_mid_points.time_from_start = rospy.Duration(3)
gest_mid.points = [gest_mid_points]


# ================== #
# Callback Functions #
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
#     time_init = rospy.get_rostime()
    # while rospy.get_rostime() < time_init + rospy.Duration(2):
    if position.data == "test":
        arm_gesture_points.positions = test
    elif position.data == "default":
        arm_gesture_points.positions = default
    elif position.data == "max_left":
        arm_gesture = gest_maxl
    elif position.data == "max_right":
        arm_gesture = gest_maxr
    elif position.data == "middle":
        arm_gesture = gest_mid
#arm_gesture = gest_def

    rospy.sleep(1)
    arm_pub.publish(arm_gesture)
    rospy.sleep(0.1)

# rospy.spin()
