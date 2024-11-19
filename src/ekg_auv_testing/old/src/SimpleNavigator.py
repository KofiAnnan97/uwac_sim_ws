#!/usr/bin/env python3
#!/usr/bin/env python3

import sys
import os
import numpy as np

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist, PoseWithCovariance, Quaternion
from gazebo_msgs.msg import ModelState, LinkStates, LinkState
from gazebo_msgs.srv import GetModelState

#from nav_lib import MapProcessor, AStar

from graphviz import Graph
from PIL import Image, ImageOps 

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import yaml
import pandas as pd

from copy import copy, deepcopy
import time

MODEL_TOPIC = 'gazebo/set_model_state'
CURRENT_MODEL_TOPIC = 'gazebo/get_model_state'

WAYPOINTS = [
    (4, 4, -18.5),
    (5, 5, -18.5),
    (6, 6, -18.5),
    (7, 8, -18.5),
    (9, 9, -18.5),
    (10, 20, -18.5),
    (3, 15, -18.5)
]

class SimpleNavigator:
    """! Navigation node class.
    This class should server as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='SimpleNavigator'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        #self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.moving_model = ModelState()


    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        rospy.Subscriber(CURRENT_MODEL_TOPIC, ModelState, self.__state_cbk)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.__link_states_cbk)
        
        # Publishers
        #self.path_pub = rospy.Publisher('/global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.model_pub = rospy.Publisher(MODEL_TOPIC, ModelState, queue_size=1)

    def __link_states_cbk(self, data):
        idx = 0
        for i in range(len(data.name)):
            if "box" in data.name[i]:
                #print(data.name[i])
                idx = i
                break
        self.moving_model.twist = data.twist[idx]
        self.moving_model.pose = data.pose[idx]
        #print(self.moving_model)
        self.ttbot_pose.header.stamp = rospy.Time.now()
        self.ttbot_pose.header.frame_id = "map"
        self.ttbot_pose.pose = data.pose[idx]
        #print(f"Box Pos := ({self.ttbot_pose.pose.position.x}, {self.ttbot_pose.pose.position.y}, {self.ttbot_pose.pose.position.z})")
    
    def get_gazebo_models(self):
        try:
            models = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            print(models)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed")

    def __state_cbk(self, data):
        self.get_gazebo_models()
        #print(data)
        #for model_state in data:
        #    if model_state.model_name == 'box':
        #        self.moving_model = model_state
        #        print(self.moving_model.pose.position)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        #rospy.loginfo('goal_pose:{:.4f},{:.4f}'.format(self.goal_pose.pose.position.x,self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        #self.ttbot_pose = data.pose
        cov = data.pose.covariance

        # Minimize covariance before execution to reduce error in map frame
        """cov_vel = Twist()
        if cov[0] > 0.2:
            cov_vel.linear.x = 0
            cov_vel.angular.z = 0.4
            self.cmd_vel_pub.publish(cov_vel)
            cov = data.pose.covariance                     # Update Covariance
        else:
            cov_vel.linear.x = 0
            cov_vel.angular.z = 0
            self.cmd_vel_pub.publish(cov_vel)
        rospy.loginfo('ttbot_pose:{:.4f},{:.4f}'.format(self.ttbot_pose.pose.position.x,self.ttbot_pose.pose.position.y))"""
        #rospy.loginfo('ttbot_pose:{}'.format(cov))

    # Map has a -90 degree roattion
    def __convert_pose_to_map(self,curr_pose):
        x_pt_res = 0.15 #0.05*384/100                          # Convert pixels to coordinate approximate for map
        y_pt_res = 0.156 #0.05*384/160
        m_x = (curr_pose.pose.position.y/-x_pt_res) + 59    # Convert x using original y value (pose)
        m_y = (curr_pose.pose.position.x/y_pt_res) + 65      # Convert y using original x value (pose)
        return round(m_x),round(m_y)                                     # Return map coordinates (as float)
    
    def __convert_map_to_pose(self, pt):
        x_pt_res = 0.156                          # Convert pixels to coordinate approximate for map
        y_pt_res = 0.15
        (x,y) = map(int, pt.split(','))                    # Retrieve pt from "int,int" format
        r_x = x_pt_res*(y - 65)                              # Convert x using given y value (map)
        r_y = -y_pt_res*(x - 59)                             # Convert x using original y value (map)
        tmp_pose = PoseStamped() 
        tmp_pose.header.stamp = rospy.Time.now()  
        tmp_pose.header.frame_id = 'map'
        tmp_pose.pose.position.x = round(r_x, 2)
        tmp_pose.pose.position.y = round(r_y, 2)
        return tmp_pose                                    # Return x and y as Pose obj

    # This method is design to return the distance between 2 points
    def __get_distance_btw_pts(self, curr_pose, next_pose):
        return math.sqrt(pow(next_pose.position.x - curr_pose.position.x, 2) + \
                        pow(next_pose.position.y - curr_pose.position.y, 2) + \
                        pow(next_pose.position.z - curr_pose.position.z, 2))

    def static_path_planner(self,waypoints):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        for pt in waypoints:
            tmp_pose = PoseStamped()
            tmp_pose.header.stamp = rospy.Time.now()
            tmp_pose.header.frame_id = 'map'
            tmp_pose.pose.position.x = pt[0]
            tmp_pose.pose.position.y = pt[1]
            tmp_pose.pose.position.z = pt[2]
            path.poses.append(tmp_pose)
        return path

    def get_path_idx(self,path,vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  vehicle_pose          PoseStamped object containing the current vehicle position.
        @return idx                   Position int the path pointing to the next goal pose to follow.
        """
        idx = 0
        closest_pt = None                                    # Store closest point for reference
        closest_pt_dist = float("inf")                       # Store closest point distance for reference
        for i in range(len(path.poses)-1):
            curr_pt = path.poses[i].pose
            #print(f"{closest_pt}\n{curr_pt.position}")
            dist = self.__get_distance_btw_pts(vehicle_pose.pose, curr_pt)
            #print(f"Closed distance: {closest_pt_dist}, Current Pt distance: {dist}")
            if dist < closest_pt_dist:
                closest_pt_dist = curr_pt
                closest_pt_dist = dist
                idx = i
        if len(path.poses) > 1:
            #idx+=1                                         # Increment to next point
            #print(f"Current idx: {idx}\nCurrent goal: ({path.poses[idx].pose.position.x}, {path.poses[idx].pose.position.y})") 
            for j in range(idx, len(path.poses)-1):
                curr_goal = path.poses[idx].pose
                next_pose = path.poses[j].pose
                if curr_goal.position.x == next_pose.position.x or \
                   curr_goal.position.y == next_pose.position.y:
                    idx = j
                else:
                    break
        #print(f"New idx: {idx}\nNew goal: ({path.poses[idx].pose.position.x}, {path.poses[idx].pose.position.y})")
        #print(f"Vehicle: ({vehicle_pose.pose.position.x}, {vehicle_pose.pose.position.y})") 
        #print(f"Closest Pt: ({closest_pt.position.x},{closest_pt.position.y}) Dist: {closest_pt_dist}")
        return idx

    def path_follower(self,vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return speed                  Desired speed
        @return heading                Desired yaw angle
        """
        speed = 0.1
        heading = 0
        pitch = 0
        #rospy.loginfo(f"\nVEHICLE: ({vehicle_pose.pose.position.x}, {vehicle_pose.pose.position.y})\nGOAL: ({current_goal_pose.position.x}, {current_goal_pose.position.y})")
        g_pos = current_goal_pose.pose.position
        v_pos = vehicle_pose.pose.position
        v_dir = vehicle_pose.pose.orientation
        curr_roll, curr_pitch, curr_yaw = euler_from_quaternion([v_dir.x, v_dir.y, v_dir.z, v_dir.w])
        desired_orientation = Quaternion()
        try:    
            dist = self.__get_distance_btw_pts(vehicle_pose.pose, current_goal_pose.pose)
            desired_yaw = math.atan2((g_pos.y - v_pos.y), (g_pos.x - v_pos.x))
            desired_pitch = math.atan2((g_pos.y - v_pos.y), dist - pow(g_pos.y - v_pos.y, 2))

            tmp_orientation = quaternion_from_euler(curr_roll, desired_pitch, desired_yaw)
            desired_orientation.x = tmp_orientation[0]
            desired_orientation.y = tmp_orientation[1]
            desired_orientation.z = tmp_orientation[2]
            desired_orientation.w = tmp_orientation[3]
            heading =  desired_yaw - curr_yaw
            pitch = desired_pitch - curr_pitch
        except:
            rospy.logerr("Desired angle could not be determined. Zero may be the denominator.")
        speed = abs(dist)
        rospy.loginfo(f"\nCurrent Pitch: {curr_pitch}\nDesired Pitch: {desired_pitch}\nCurrent Yaw: {curr_yaw}\nDesired Yaw: {desired_yaw}\n---------------------------")
        return speed,heading, pitch

    def move_ttbot(self,speed,heading, pitch):
        #print(speed, heading, pitch)
        ms = ModelState()
        ms.model_name = "box"
        rospy.wait_for_service('/gazebo/set_model_state')
        ms.pose = self.ttbot_pose.pose
        k_ph = 0.2
        k_pp = 0.1
        range = 0.3
        MAX_SPEED = 0.12
        cmd_vel = Twist()
        if pitch > range:
            cmd_vel.linear.x = 0
            cmd_vel.angular.y = min(MAX_SPEED, k_pp*pitch)
            cmd_vel.angular.z = 0
        elif pitch < range:
            cmd_vel.linear.x = 0
            cmd_vel.angular.y = max(-MAX_SPEED, k_pp*pitch)
            cmd_vel.angular.z = 0
        elif heading > range:                                     
            cmd_vel.linear.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = max(-MAX_SPEED, k_ph*heading)   # Turn right
        elif heading < -range:                                  
            cmd_vel.linear.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = min(MAX_SPEED, k_ph*heading)    # Turn left
        else:
            cmd_vel.linear.x = min(0.05, speed)
            cmd_vel.angular.y = 0                       # Go straight
            cmd_vel.angular.z = 0

        ms.twist = cmd_vel
        ms.pose.position.x += ms.twist.linear.x
        ms.pose.position.y += ms.twist.linear.y
        ms.pose.position.z += ms.twist.linear.z
        ms.pose.orientation.y += ms.twist.angular.y
        ms.pose.orientation.z += ms.twist.angular.z
        #norient = quaternion_from_euler(0, l)

        #rospy.loginfo(f"\nSpeed: {ms.twist.linear.x}\nPitch: {ms.twist.angular.y}\n Yaw: {ms.twist.angular.z}")
        #print(ms.twist)
        self.model_pub.publish(ms)
        self.cmd_vel_pub.publish(cmd_vel)
        

    '''
    Main loop
    '''
    def run(self):
        ms = ModelState()
        ms.model_name = "box"

        #last_goal = self.goal_pose
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        #path_complete = False
        #timeout = False
        path = self.static_path_planner(WAYPOINTS)
        #print(path)

        while not rospy.is_shutdown():
            # 1. Create the path to follow
            # Check to see if new goal has been given 
            #if self.goal_pose.pose.position.x != last_goal.pose.position.x or \
            #   self.goal_pose.pose.position.y != last_goal.pose.position.y:
                #rospy.loginfo(f"\n== LAST GOAL ==\n{last_goal.pose.position}\n== CURRENT GOAL ==\n{self.goal_pose.pose.position}")
            
            #    self.path_pub.publish(path)
                #self.print_path_to_file(path)
                #last_goal = self.goal_pose
            # 2. Loop through the path and move the robot
            if len(path.poses) > 0:
                idx = self.get_path_idx(path,self.ttbot_pose)
                current_goal = path.poses[idx]
                #rospy.loginfo(f"\nRobot Pos: ({self.ttbot_pose.pose.position.x},{self.ttbot_pose.pose.position.y},{self.ttbot_pose.pose.position.z})\nCurrent goal: ({current_goal.pose.position.x}, {current_goal.pose.position.y},{current_goal.pose.position.z})")
                speed,heading, pitch = self.path_follower(self.ttbot_pose,current_goal)
                # Provide a threshold that robot can be within to reach goal position
                THRESHOLD = 0.08
                """if math.fabs(self.ttbot_pose.pose.position.x - self.goal_pose.pose.position.x) <= THRESHOLD and \
                   math.fabs(self.ttbot_pose.pose.position.y - self.goal_pose.pose.position.y) <= THRESHOLD and \
                   math.fabs(self.ttbot_pose.pose.position.z - self.goal_pose.pose.position.z) <= THRESHOLD:
                    print("Complete")
                    self.move_ttbot(0,0,0)                          # Set robot movement to 0
                else:"""
                self.move_ttbot(speed,heading,pitch)                # Set robot movement using speed and heading
            self.rate.sleep()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))

if __name__ == "__main__":
    simple_nav = SimpleNavigator(node_name='SimpleNavigator')
    simple_nav.init_app()
    try:
        simple_nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)