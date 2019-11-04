#!/usr/bin/env python
import rospy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

# initialize the node for hand movement
rospy.init_node('hand_mover')

# different string commands:
# odds, evens, rock, paper, scissors
action = String()
# begin the callback function to get the action to perform
def gesture_callback(data):
    global action
    action.data = data.data

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

# initialize "odds" and assign hand_array
odds = JointTrajectory()
odds.joint_names = hand_array
# set points for "odds" finger positions, only has pointer finger extend
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
odds_points = JointTrajectoryPoint()
odds_points.positions = []
odds_points.velocities = []*
# send points array back to "odds" for eventual loop commands
odds.points = [odds_points]


# initialize "evens" and assign hand_array
evens = JointTrajectory()
evens.joint_names = hand_array
# set points for "evens" finger positions, extends pointer/middle finger
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
evens_points = JointTrajectoryPoint()
evens_points.positions = []
evens_points.velocities = []*
# send points array back to "evens" for eventual loop commands
evens.points = [evens_points]


# *****************************************************************************
#                         setup for Rock-Paper-Scissors
# *****************************************************************************

# initialize "rock" and assign hand_array
rock = JointTrajectory()
rock.joint_names = hand_array
# set points for "rock" finger positions, creates a fist
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
rock_points = JointTrajectoryPoint()
rock_points.positions = []
rock_points.velocities = []*
# send points array back to "rock" for eventual loop commands
rock.points = [rock_points]


# initialize "paper" and assign hand_array
paper = JointTrajectory()
paper.joint_names = hand_array
# set points for "paper" finger positions, extends all fingers
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
paper_points = JointTrajectoryPoint()
paper_points.positions = []
paper_points.velocities = []*
# send points array back to "paper" for eventual loop commands
paper.points = [paper_points]


# NOTE: this is the same as "evens"
# initialize "scissors" and assign hand_array
scissors = JointTrajectory()
scissors.joint_names = hand_array
# set points for "scissors" finger positions, extends pointer/middle finger
# JointTrajectoryPoint of 0 = fully extended, 1 = fully retracted
scissors_points = JointTrajectoryPoint()
scissors_points.positions = []
scissors_points.velocities = []*
# send points array back to "scissors" for eventual loop commands
scissors.points = [scissors_points]


# lets get loopy
while not rospy.is_shutdown():
    if action.data == "odds":
        time_init = rospy.get_rostime()
        while rospy.get_rostime() < time_init+rospy.Duration(1):
            odds.header.stamp = rospy.get_rostime()
            hand_pub.publish(odds)
            rospy.sleep(0.1)
    elif action.data == "evens":
        time_init = rospy.get_rostime()
        while rospy.get_rostime() < time_init+rospy.Duration(1):
            evens.header.stamp = rospy.get_rostime()
            hand_pub.publish(evens)
            rospy.sleep(0.1)
    elif action.data == "rock":
        time_init = rospy.get_rostime()
        while rospy.get_rostime() < time_init+rospy.Duration(1):
            rock.header.stamp = rospy.get_rostime()
            hand_pub.publish(rock)
            rospy.sleep(0.1)
    elif action.data == "paper":
        time_init = rospy.get_rostime()
        while rospy.get_rostime() < time_init+rospy.Duration(1):
            paper.header.stamp = rospy.get_rostime()
            hand_pub.publish(paper)
            rospy.sleep(0.1)
    elif action.data == "scissors":
        time_init = rospy.get_rostime()
        while rospy.get_rostime() < time_init+rospy.Duration(1):
            scissors.header.stamp = rospy.get_rostime()
            hand_pub.publish(scissors)
            rospy.sleep(0.1)
