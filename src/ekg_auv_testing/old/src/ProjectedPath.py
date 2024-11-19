#!/usr/bin/env python3
import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Path
from rospy_message_converter import message_converter, json_message_converter
from ekg_auv_testing.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, \
                                bcn_remote_gps, bcn_status_array, bcn_status, head, \
                                IverOSI, loc
from StringtoROSmsg import StringtoROSmsg
import json
import math

MODEL_TOPIC = 'gazebo/set_model_state'
PATH_TOPIC = "/seatrac_usbl/projected_path"
VELOCITY_TOPIC = '/cmd_vel'
TRANSPONDER_TOPIC = "/USBL/transponder_manufacturer_1/command_request"
TRANSCEIVER_TOPIC = "/USBL/transceiver_manufacturer_168/command_response"

COMMON_TOPIC = '/USBL/common_interrogation_ping'

TEAM_SIZE = 2

#Transceiver
class ProjectedPath():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.path = Path()
        self.recieved_value = None
        self.neighbor_paths = dict()
        self.recieved_path = Path()

        #Initialize Subsriber
        rospy.Subscriber(PATH_TOPIC, String, self.__path_cbk)
        rospy.Subscriber(TRANSCEIVER_TOPIC, String, self.__usbl_response_cbk)
        rospy.Subscriber(COMMON_TOPIC, String, self.__comm_cbk)
        
        #Initialize Publisher
        self.move_pub = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=1)
        self.model_pub = rospy.Publisher(MODEL_TOPIC, ModelState, queue_size=1)

    def __path_cbk(self, msg):
        if msg is not None:
            json_str = msg.data
            strm = StringtoROSmsg()
            self.recieved_path = strm.rosmsg_from_str(json_str)
        #rospy.loginfo(f"\n=== RECIEVING PROJECTED PATH ===\n{self.recieved_path}")
        

    def __usbl_response_cbk(self, msg):
        self.recieved_value = msg
        #self.recieved_value = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)

    def __comm_cbk(self, msg):
        if msg is not None:
            json_str = msg.data
            json_obj = json.loads(json_str)
            model_name = json_obj['model_name']
            del json_obj['model_name']
            json_str = json.dumps(json_obj)
            strm = StringtoROSmsg()
            self.neighbor_paths[model_name] = strm.rosmsg_from_str(json_str)
            #print(self.neighbor_paths)

    def __dist_between_points(self, start_pose, end_pose):
        x1 = start_pose.position.x
        y1 = start_pose.position.y
        z1 = start_pose.position.z
        x2 = end_pose.position.x
        y2 = end_pose.position.y
        z2 = end_pose.position.z
        dist = 0
        sum_of_squares = math.pow(x1-x2,  2)+ math.pow(y1-y2, 2) + math.pow(z1-z2, 2) 
        dist = math.sqrt(sum_of_squares)
        return dist

    def __get_closest_waypoint(self, auv_pose, path):
        closest_pt = None
        closest_dist = float("inf")
        for pt in path:
            dist = self.__dist_between_points(auv_pose.pose, pt)
            if closest_pt is None:
                closest_pt = pt
                closest_dist = dist
            else:
                if dist < closest_dist:
                    closest_pt = pt
                    closest_dist = dist
        return closest_pt
    
    def wait_for_request(self):
        if not rospy.wait_for_message(TRANSPONDER_TOPIC, String, timeout=rospy.Duration(10)):
            return "fail"
        return "success"
    
    def get_neighbor_paths(self):
        self.neighbor_paths['test'] = self.recieved_path
        #print(self.neighbor_path['test'])
        #self.get_neighbor_paths[neighbor_path.model_name] = self.recieved_value

    def run(self):
        while not rospy.is_shutdown():
            #msg_status = self.wait_for_request()
            #if msg_status == "success":
            #self.get_neighbor_paths()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("projected_path_node", anonymous=True)
    p_node = ProjectedPath()
    try:
        p_node.run()
    except KeyboardInterrupt:
        print("Shutting down node")
