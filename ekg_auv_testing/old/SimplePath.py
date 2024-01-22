#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import actionlib
from tf import TransformListener
import time
import json
from math import sqrt, pow
from rospy_message_converter import message_converter, json_message_converter
from ekg_auv_testing.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, \
                                bcn_remote_gps, bcn_status_array, bcn_status, head, \
                                IverOSI, loc

MODEL_TOPIC = 'gazebo/set_model_state'
CURRENT_MODEL_TOPIC = 'gazebo/model_states'
VELOCITY_TOPIC = '/cmd_vel'
PATH_TOPIC = '/seatrac_usbl/projected_path' 
TRANSPONDER_TOPIC = "/USBL/transponder_manufacturer_1/command_request"
TRANSCEIVER_TOPIC = "/USBL/transceiver_manufacturer_168/command_response"

# Transciever Topics
INTERROGATION_TOPIC = '/USBL/transceiver_manufacturer_168/interrogation_mode'
CHANNEL_SWITCH_TOPIC = '/USBL/transceiver_manufacturer_168/channel_switch'

# Transponder Topics
COMMON_TOPIC = '/USBL/common_interrogation_ping'
INDIVIDUAL_TOPIC = '/USBL/transponder_manufacturer_1/individual_interrogation_ping'

WAYPOINTS = [
    (4, 4, -18.5),
    (5, 5, -18.5),
    (6, 6, -18.5),
    (7, 8, -18.5),
    (9, 9, -18.5),
    (10, 20, -18.5),
    (3, 15, -18.5)
]

#Transponder
class SimplePath:
    def __init__(self):
        self.rate = rospy.Rate(0.1)
        self.goal_pose = PoseStamped()
        self.curr_pose = PoseStamped()
        self.dist_tolerance = 0.5
        self.vel = Twist()
        
        self.waypoints = Path()
        global WAYPOINTS
        for pt in WAYPOINTS:
            tmp_pose = PoseStamped()
            tmp_pose.header.frame_id = "map"
            tmp_pose.pose.position.x = pt[0]
            tmp_pose.pose.position.y = pt[1]
            tmp_pose.pose.position.z = pt[2]
            self.waypoints.poses.append(tmp_pose)

        #Initialize Publishers
        self.model_pub = rospy.Publisher(MODEL_TOPIC, ModelState, queue_size=1)
        self.path_pub = rospy.Publisher(PATH_TOPIC, String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.channel_pub = rospy.Publisher(CHANNEL_SWITCH_TOPIC, String, queue_size=1)
        self.common_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=1)

        #Initialize Subscribers
        rospy.Subscriber(VELOCITY_TOPIC, Twist, self.__vel_cbk)
        rospy.Subscriber(CURRENT_MODEL_TOPIC, ModelState, self.__state_cbk)

        # Action Client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()
        if not self.client.wait_for_server(rospy.Duration(60)):
            rospy.logerr("Action Server not available!")
            return
            #rospy.signal_shutdown("Shutting down")
        rospy.loginfo("Connecting to Action Server")

    def __vel_cbk(self, data):
        self.vel = data
        rospy.loginfo(self.vel)
        
    def run_simple_box(self, model_state):
        diagonal = sqrt(0.5)   # diagonal components for 1 m/s speed
        while not rospy.is_shutdown():
            for (model_state.twist.linear.x, \
                model_state.twist.linear.y, \
                model_state.twist.linear.z) in \
                ((1.0, 0.0, -0.4), (diagonal, diagonal, 0.0), \
                (0.0, 1.0, 0.2), (-diagonal, diagonal, 0.0), \
                (-1.0, 0.0, -0.3), (-diagonal, -diagonal, 0.0), \
                (0.0, -1.0, 0.1), (diagonal, -diagonal, 0.0)):
                self.model_pub.publish(model_state)
                self.rate.sleep()

    def follow_waypoint(self, vehicle_state, current_goal):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position = current_goal.pose.position
        goal_pose.target_pose.pose.orientation = current_goal.pose.orientation
        #rospy.loginfo(f"\n== Send Current Goal Pose == \n{goal_pose.target_pose.pose.position}")
        self.client.send_goal(goal_pose)
        dist = float("inf")
        while(dist > self.dist_tolerance):
            self.client.wait_for_result(rospy.Duration(4.0))
            curr_pose = vehicle_state.pose
            dist = self.__get_distance_btw_pts(curr_pose, current_goal.pose)
            #print(f"\nDistance to waypoint: {dist}")

    def get_project_path(self, vehicle_state, idx, num_of_pts):
        path_length = num_of_pts
        project_path = Path()
        start_pose = PoseStamped()
        start_pose.pose = vehicle_state.pose
        #start_pose.header = vehicle_state.header
        project_path.poses.append(start_pose)

        for j in range(idx, idx+path_length):
            if j < len(self.waypoints.poses):
                project_path.poses.append(self.waypoints.poses[j])
            else:
                break
        return project_path
    
    def __get_distance_btw_pts(self, curr_pose, next_pose):
        return sqrt(pow(next_pose.position.x - curr_pose.position.x, 2) + \
                        pow(next_pose.position.y - curr_pose.position.y, 2) + \
                        pow(next_pose.position.z - curr_pose.position.z, 2))

    def get_closest_point(self, vehicle_state, path):
        idx = 0
        closest_pt = None
        closest_pt_dist = float("inf")
        for i in range(len(path.poses)):
            waypoint = path.poses[i]
            dist = self.__get_distance_btw_pts(vehicle_state.pose, waypoint.pose)
            if dist < closest_pt_dist:
                closest_pt = waypoint
                closest_pt_dist = dist;
                idx = i
        return idx

    def send_projected_status(self, vehicle_state, idx, num_of_pts):
        path = self.get_project_path(vehicle_state, idx, num_of_pts)
        #sent_state = vehicle_status()
        #sent_state.model_name = vehicle_state.model_name
        model_twist = vehicle_state.twist
        #sent_state.next_poses = path.poses
        json_str = json_message_converter.convert_ros_message_to_json(path)
        json_obj = json.loads(json_str)
        msg_name = {"rosmsg_name": "Path"}
        json_obj.update(msg_name)
        json_str = json.dumps(json_obj)
        msg = String()
        msg.data = str(json_str)
        #rospy.loginfo(f"\n=== SENDING PROJECTED PATH ===\n{msg.data}")
        self.path_pub.publish(msg)

    def ping_all_projected_path(self, vehicle_state, idx, num_of_pts):
        msg = String()
        msg.data = "common"
        self.channel_pub.publish(msg)

        path = self.get_project_path(vehicle_state, idx, num_of_pts)
        json_str = json_message_converter.convert_ros_message_to_json(path)
        json_obj = json.loads(json_str)
        msg_name = {"rosmsg_name": "Path"}
        json_obj.update(msg_name)
        json_obj.update({"model_name": vehicle_state.model_name})
        json_str = json.dumps(json_obj)
        msg.data = str(json_str)
        #rospy.loginfo(f"\n=== SENDING PROJECTED PATH ===\n{msg.data}")
        self.common_pub.publish(msg)

    def run(self):
        ms = ModelState()
        ms.model_name = "box"
        rospy.wait_for_service('/gazebo/set_model_state')
        
        while not rospy.is_shutdown():
            path = self.waypoints
            idx = self.get_closest_point(ms, path)
            curr_waypoint = path.poses[idx]
            #self.follow_waypoint(ms, waypoint)
            #self.send_projected_status(ms, idx, 3)
            self.ping_all_projected_path(ms, idx, 3)
            self.rate.sleep()

        #self.run_simple_box(ms)

if __name__ == '__main__':
    rospy.init_node("simple_path", anonymous=True)
    sp = SimplePath()
    try:
        sp.run()
    except rospy.ROSInterruptException:
        print("Shutting down")