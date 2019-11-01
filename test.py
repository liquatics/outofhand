#!/usr/bin/env python
import rospy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

# initialize the node
rospy.init_node('arm_mover')
