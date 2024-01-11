#!/usr/bin/env python3

import rospy
import sys
import os
import time

import warnings
import re

from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, Twist
import tf.transformations as tft

class SpawnBeacon():
    def __init__(self):
        pass 

