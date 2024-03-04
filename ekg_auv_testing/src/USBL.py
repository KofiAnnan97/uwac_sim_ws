#!/usr/bin/env python3

import math
import time
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState

from rospy_message_converter import message_converter, json_message_converter

from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim

INTERROGATION_MODE = ['common', 'individual']

GAZEBO_GET_MODEL_STATE_TOPIC = 'gazebo/get_model_state'
GAZEBO_MODEL_STATES_TOPIC = 'gazebo/model_states'

SPEED_OF_SOUND = 1540                                     # speed of sound underwater

# Channel Index
CHANNELS_IDS = {
    "1": "glider_1",
    "168": "beacon_1",
    "169": "beacon_2"
}

class Transceiver():
    """def __init__(self, transponder_id, transponder_model, transceiver_id, transceiver_model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.mode = INTERROGATION_MODE[0]
        self.transceiver_model = transceiver_model
        self.transponder_model = transponder_model
        self.tx_pose = PoseStamped()

        self.models = ModelStates()

        self.common_data = None
        self.transponder_location = Point()

        # Topics
        TRANSPONDER_LOCATION_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/transponder_pose'
        CHANNEL_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/channel_switch'
        MODE_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/mode'
        COMMON_TOPIC = '/USBL/common_ping'

        RESPONSE_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/command_request'
        REQUEST_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'

        # Gazebo Subscribers
        #rospy.Subscriber(GAZEBO_GET_MODEL_STATE_TOPIC, ModelStates, self.__model_cbk)
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__models_cbk)

        # ROS Publishers
        self.channel_pub = rospy.Publisher(CHANNEL_TOPIC, String, queue_size=1)
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        self.comm_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=10)
        self.tx_loc_pub = rospy.Publisher(TRANSPONDER_LOCATION_TOPIC, Point, queue_size=20)
        self.resp_pub = rospy.Publisher(RESPONSE_TOPIC, USBLResponseSim, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(CHANNEL_TOPIC, String, self.__channel_cbk)
        rospy.Subscriber(MODE_TOPIC, String, self.__mode_cbk)
        rospy.Subscriber(TRANSPONDER_LOCATION_TOPIC, Point, self.__rx_loc_cbk)
        # self.requests = rospy.Subscriber(REQUEST_TOPIC, USBLRequestSim, self.__request_cbk)
        """

    def __init__(self, transponder_id, transceiver_id, model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.mode = INTERROGATION_MODE[0]
        self.transceiver_model = model
        self.transponder_model = None
        self.tx_pose = PoseStamped()

        self.models = ModelStates()

        self.common_data = None
        self.transponder_location = PoseStamped()

        # Topics
        TRANSPONDER_LOCATION_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/transponder_pose'
        CHANNEL_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/channel_switch'
        MODE_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/mode'
        COMMON_TOPIC = '/USBL/common_ping'

        REQUEST_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/command_request'
        RESPONSE_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'

        # Gazebo Subscribers
        #rospy.Subscriber(GAZEBO_GET_MODEL_STATE_TOPIC, ModelStates, self.__model_cbk)
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__models_cbk)

        # ROS Publishers
        self.channel_pub = rospy.Publisher(CHANNEL_TOPIC, String, queue_size=1)
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        self.comm_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=10)
        self.tx_loc_pub = rospy.Publisher(TRANSPONDER_LOCATION_TOPIC, PoseStamped, queue_size=20)
        self.resp_pub = rospy.Publisher(RESPONSE_TOPIC, USBLResponseSim, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(CHANNEL_TOPIC, String, self.__channel_cbk)
        rospy.Subscriber(MODE_TOPIC, String, self.__mode_cbk)
        rospy.Subscriber(TRANSPONDER_LOCATION_TOPIC, PoseStamped, self.__rx_loc_cbk)
        rospy.Subscriber(REQUEST_TOPIC, USBLRequestSim, self.__request_cbk)


    # Callback Functions

    def __rx_loc_cbk(self, data):
        if data is not None:
            self.transponder_location = data
            #rospy.loginfo(f"[Transciever {self.transceiver_id}]: Transponder location at ({data.pose.position.x}, {data.pose.position.y}, {data.pose.position.z})")

    def __common_cbk(self, data):
        pass

    def __models_cbk(self, data):
        if data is not None:
            self.models = data
            self.tx_pose.pose = self.get_model_pos(self.transceiver_model)

    """def __model_cbk(self, data):
        try:
            print(data)
            if data.model_name == self.transponder_model:
                self.tx_pose.pose = data.pose
                self.tx_pose.header = data.header
        except rospy.ServiceException as e:
           pass"""

    def __channel_cbk(self, data):
        msg = data
        if msg is not None:
            self.transponder_id = msg.data 

    def __mode_cbk(self, data):
        msg = data
        if msg is not None:
            if msg.data == INTERROGATION_MODE[0]:     # Common
                self.mode = msg.data
            elif msg.data == INTERROGATION_MODE[1]:   # Individual
                self.mode = msg.data
            else:
                rospy.loginfo(f"{msg.data} is not a proper mode.")
    
    def __request_cbk(self, data):
        if data is not None:
            if data.data == "location":
                self.send_ping()
                self.send_command_response()
                
    # Methods
    def send_ping(self):
        msg = String()
        msg.data = 'ping'
        if self.mode == INTERROGATION_MODE[0]:       # Common channel
            self.comm_pub.publish(msg)
        elif self.mode == INTERROGATION_MODE[1]:     # Inidividual channel
            # rospy.loginfo(f'/USBL/transponder_{self.transponder_id}/ping')
            # rospy.loginfo(msg)
            individual_pub = rospy.Publisher(f'/USBL/transponder_{self.transponder_id}/ping', String, queue_size=1)
            individual_pub.publish(msg)

    def switch_channel(self, channel_id):
        if channel_id != int(self.transponder_id):
            REQUEST_TOPIC = f'/USBL/transponder_{self.transceiver_id}/command_request'
            self.requests = rospy.Subscriber(REQUEST_TOPIC, USBLRequestSim, self.__request_cbk)
        self.transponder_id = channel_id
        # rospy.loginfo(f"Transceiver mode: {self.transponder_id}")

    def set_interrogation_mode(self, mode):
        if mode == "common":
            self.mode = INTERROGATION_MODE[0]
        elif mode == "individual":
            self.mode = INTERROGATION_MODE[1]
        else:
            rospy.loginfo(f"{mode} is not a proper mode.")
            return
        # rospy.loginfo(f"Transceiver mode: {self.mode}")

    def get_model_pos(self, model_name):
        try:
            idx = -1
            for i in range(len(self.models.name)):
                if self.models.name[i] == model_name:
                    idx = i
                    break
            return self.models.pose[idx]
        except Exception:
           rospy.loginfo(f"Model '{model_name}' was not found.")
           return None
        
    def send_command_response(self):
        msg = USBLResponseSim()
        msg.transceiverID = int(self.transceiver_id)
        msg.transceiverModelName = self.transceiver_model
        msg.responseID = int(self.transponder_id)

        loc_str = json_message_converter.convert_ros_message_to_json(self.transponder_location)
        msg.data = loc_str
        # print(msg)
        self.resp_pub.publish(msg)

    def send_msg(self, msg):
        if self.mode == INTERROGATION_MODE[0]:
            self.comm_pub.publish(msg)
        elif self.mode == INTERROGATION_MODE[1]:
            individual_pub = rospy.Publisher(f'/USBL/transponder_{self.transponder_id}/ping', String, queue_size=1)
            individual_pub.publish(msg)



class Transponder():
    def __init__(self, transponder_id, transceiver_id, model_name): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.transceiver_model = None
        self.transponder_model = model_name
        self.rx_pose = PoseStamped()

        self.models = ModelStates()

        self.command_resp = USBLResponseSim()

        self.mu = 0
        self.sigma = 0.9

        # Topics
        COMMON_TOPIC = '/USBL/common_ping'
        INDIVIDUAL_TOPIC = f'/USBL/transponder_{self.transponder_id}/ping'
        RESPONSE_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'
        REQUEST_TOPIC= f'/USBL/transceiver_{self.transceiver_id}/command_request'

        # Gazebo Subscribers
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__models_cbk)

        # ROS Publishers
        #self.loc_pub = rospy.Publisher(self.TRANSPONDER_LOCATION_TOPIC, Point, queue_size=10)
        self.request_pub = rospy.Publisher(REQUEST_TOPIC, USBLRequestSim, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(INDIVIDUAL_TOPIC, String, self.__individual_cbk)
        rospy.Subscriber(RESPONSE_TOPIC, USBLResponseSim, self.__resp_cbk)

    """def __init__(self, transponder_id, transponder_model, transceiver_id, transceiver_model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.transceiver_model = transceiver_model
        self.transponder_model = transponder_model
        self.rx_pose = PoseStamped()

        self.models = ModelStates()

        self.command_resp = USBLResponseSim()

        self.mu = 0
        self.sigma = 0

        # Topics
        COMMON_TOPIC = '/USBL/common_ping'
        INDIVIDUAL_TOPIC = f'/USBL/transponder_{self.transponder_id}/ping'
        REQUEST_TOPIC = f'/USBL/transponder_{self.transceiver_id}/command_request'
        RESPONSE_TOPIC = f'/USBL/transceiver_{self.transponder_id}/command_response'

        # Gazebo Subscribers
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__models_cbk)

        # ROS Publishers
        #self.loc_pub = rospy.Publisher(self.TRANSPONDER_LOCATION_TOPIC, Point, queue_size=10)
        self.request_pub = rospy.Publisher(REQUEST_TOPIC, USBLRequestSim, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(INDIVIDUAL_TOPIC, String, self.__individual_cbk)
    """

    def __common_cbk(self, data):
        msg = data
        if msg is not None:
            if msg.data == 'ping':
                # rospy.loginfo(f'Pinging {self.transponder_id}')
                self.query_location()

    def __individual_cbk(self, data):
        msg = data
        if msg.data == 'ping':
            # rospy.loginfo(f'Pinging {self.transponder_id}')
            self.query_location()

    def __models_cbk(self, data):
        if data is not None:
            self.models = data
            self.rx_pose.pose = self.get_model_pos(self.transponder_model)

    def __resp_cbk(self, data):
        if data is not None:
            #print(data)
            self.command_resp = data
    
    # Methods
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

    def send_location_request(self):
        req = USBLRequestSim()
        req.responseID = int(self.transponder_id)
        req.transceiverID = int(self.transceiver_id)
        req.transponderModelName = self.transponder_model
        req.data = "location"
        #print(req)
        self.request_pub.publish(req)

    def set_tx_channel(self, channel):
        self.transceiver_id = channel
    
    def get_response(self):
        return self.command_resp

    def get_model_pos(self, model_name):
        try:
            idx = -1
            for i in range(len(self.models.name)):
                if self.models.name[i] == model_name:
                    idx = i
            # print(data.pose[idx])
            return self.models.pose[idx]
        except Exception:
           rospy.loginfo(f"Model '{model_name}' was not found.")
           return None

    # FIX: MAKE SURE THAT THE MODEL IS ACTUALLY GRABS THE TRANSCEIVER MODEL LOCATION 
    def query_location(self):
        try:
            """ models = rospy.ServiceProxy(GAZEBO_MODEL_STATES_TOPIC, ModelStates)
            resp = models(self.transceiver_model, self.transceiver_model)
            rospy.loginfo(resp)
            
            tx_pose.header = resp.header
            tx_pose.pose = resp.pose"""
           
            tx_pose = PoseStamped()
            tx_pose.pose = self.get_model_pos(self.transceiver_model)

            dist = self.__dist_between_points(self.rx_pose.pose, tx_pose.pose)
            # rospy.loginfo(f"Calculated distance: {dist}")
            sound_propagation_speed = SPEED_OF_SOUND + self.rx_pose.pose.position.z/1000 * 17
            delay = dist/sound_propagation_speed
            # rospy.loginfo(f"Delay time: {delay}")
            time.sleep(delay)
            
            loc_pub = rospy.Publisher(f'/USBL/transceiver_{self.transceiver_id}/transponder_pose', PoseStamped, queue_size=10)
            tx_loc = PoseStamped()
            noise = np.random.normal(self.mu, self.sigma, 3)
            tx_loc.header.stamp = rospy.Time.now()
            tx_loc.pose.position.x = self.rx_pose.pose.position.x + noise[0]
            tx_loc.pose.position.y = self.rx_pose.pose.position.y + noise[1]
            tx_loc.pose.position.z = self.rx_pose.pose.position.z + noise[2]
            loc_pub.publish(tx_loc)
                    
        except Exception as e:
            print(e)
            # rospy.loginfo("Could not find location")

###########
# TESTING #
###########

"""if __name__ == "__main__":
    rospy.init_node("usbl_comm_testing", anonymous=True)

    transponder_id = "1"
    transponder_model = "sphere1"
    transceiver_id = "168"
    transceiver_model = "box1"

    #tx1 = Transceiver(transponder_id, transponder_model, transceiver_id, transceiver_model)
    #tx2 = Transceiver(transponder_id, transponder_model, "169", "box2")
    #rx1 = Transponder(transponder_id, transponder_model, transceiver_id, transceiver_model)

    tx1 = Transceiver(transponder_id, transceiver_id, "box1")
    tx2 = Transceiver(transponder_id, "169", "box2")
    rx1 = Transponder(transponder_id, transceiver_id, "sphere1")
    
    rx1.mu = 0.2
    rx1.sigma = 0.07
    
    while not rospy.is_shutdown():
        time.sleep(2)
        tx1.set_interrogation_mode("individual")
        tx1.switch_channel("1")
        rx1.send_location_request()

        rx1.transceiver_id = "169"
        rx1.transceiver_model = "box2"

        time.sleep(2)
        tx2.set_interrogation_mode("individual")
        tx2.switch_channel("1")
        rx1.send_location_request()

        rx1.transceiver_id = "168"
        rx1.transceiver_model = "box1"
        
        rospy.Rate(10).sleep()"""