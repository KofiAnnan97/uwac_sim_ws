#!/usr/bin/env python3

import math
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState

INTERROGATION_MODE = ['common', 'individual']

GAZEBO_MODEL_STATES_TOPIC = 'gazebo/get_model_state'

SPEED_OF_SOUND = 1540                                     # speed of sound underwater

# Channel Index
CHANNELS_IDS = {
    "1": "glider_1",
    "168": "beacon_1",
    "169": "beacon_2"
}

class Transceiver():
    def __init__(self, transponder_id, transponder_model, transceiver_id, transceiver_model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.mode = INTERROGATION_MODE[0]
        self.transceiver_model = transceiver_model
        self.transponder_model = transponder_model
        self.tx_pose = PoseStamped()

        self.common_data = None

        # Topics
        TRANSPONDER_LOCATION_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/transponder_pose'
        CHANNEL_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/channel_switch'
        MODE_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/mode'
        COMMON_TOPIC = '/USBL/common_ping'

        # Gazebo Subscribers
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__model_cbk)

        # ROS Publishers
        self.channel_pub = rospy.Publisher(CHANNEL_TOPIC, String, queue_size=1)
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        self.comm_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=10)
        self.tx_loc_pub = rospy.Publisher(TRANSPONDER_LOCATION_TOPIC, Point, queue_size=20)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(CHANNEL_TOPIC, String, self.__channel_cbk)
        rospy.Subscriber(MODE_TOPIC, String, self.__mode_cbk)
        rospy.Subscriber(TRANSPONDER_LOCATION_TOPIC, Point, self.__rx_loc_cbk)

    def __rx_loc_cbk(self, data):
        pass

    def __common_cbk(self, data):
        pass

    def __model_cbk(self, data):
        try:
            print(data)
            if data.model_name == self.transponder_model:
                self.tx_pose.pose = data.pose
                self.tx_pose.header = data.header
        except rospy.ServiceException as e:
           pass

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
    
    # Methods
    def set_init_pose(self, x, y, z):
        self.tx_pose.pose.position.x = x 
        self.tx_pose.pose.position.y = y
        self.tx_pose.pose.position.z = z 

    def send_ping(self):
        msg = String()
        msg.data = 'ping'
        if self.mode == INTERROGATION_MODE[0]:       # Common channel
            self.comm_pub.publish(msg)
        elif self.mode == INTERROGATION_MODE[1]:     # Inidividual channel
            rospy.loginfo(f'/USBL/transponder_{self.transponder_id}/ping')
            rospy.loginfo(msg)
            individual_pub = rospy.Publisher(f'/USBL/transponder_{self.transponder_id}/ping', String, queue_size=1)
            individual_pub.publish(msg)

    def switch_channel(self, channel_id):
        self.transponder_id = channel_id
        rospy.loginfo(f"Transceiver mode: {self.transponder_id}")

    def set_interrogation_mode(self, mode):
        if mode == "common":
            self.mode = INTERROGATION_MODE[0]
        elif mode == "individual":
            self.mode = INTERROGATION_MODE[1]
        else:
            rospy.loginfo(f"{mode} is not a proper mode.")
        rospy.loginfo(f"Transceiver mode: {self.mode}")

    def send_location(self):
        
        pass

class Transponder():
    def __init__(self, transponder_id, transponder_model, transceiver_id, transceiver_model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.transceiver_model = transceiver_model
        self.transponder_model = transponder_model
        self.rx_pose = PoseStamped()

        # Topics
        TRANSPONDER_LOCATION_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/transponder_pose'
        COMMON_TOPIC = '/USBL/common_ping'
        INDIVIDUAL_TOPIC = f'/USBL/transponder_{self.transponder_id}/ping'

        # Gazebo Subscribers
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__model_cbk)

        # ROS Publishers
        self.loc_pub = rospy.Publisher(TRANSPONDER_LOCATION_TOPIC, Point, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(INDIVIDUAL_TOPIC, String, self.__individual_cbk)

    def __common_cbk(self, data):
        msg = data
        if msg is not None:
            if msg.data == 'ping':
                rospy.loginfo(f'Pinging {self.transponder_id}')
                self.query_location()

    def __individual_cbk(self, data):
        msg = data
        if msg.data == 'ping':
            rospy.loginfo(f'Pinging {self.transponder_id}')
            self.query_location()

    def __model_cbk(self, data):
        try:
            print(data)
            if data.model_name == self.transponder_model:
                self.rx_pose.pose = data.pose
                self.rx_pose.header = data.header
        except rospy.ServiceException as e:
           pass
    
    # Methods
    def set_init_pose(self, x, y, z):
        self.rx_pose.pose.position.x = x 
        self.rx_pose.pose.position.y = y
        self.rx_pose.pose.position.z = z   
    
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

    # FIX: MAKE SURE THAT THE MODEL IS ACTUALLY GRABS THE TRANSCEIVER MODEL LOCATION 
    def query_location(self):
        try:
            models = rospy.ServiceProxy(GAZEBO_MODEL_STATES_TOPIC, GetModelState)
            resp = models(self.transceiver_model, self.transceiver_model)
            rospy.loginfo(resp)
            tx_pose = PoseStamped()
            tx_pose.header = resp.header
            tx_pose.pose = resp.pose
            dist = self.__dist_between_points(self.rx_pose.pose, tx_pose.pose)
            rospy.loginfo(f"Calculated distance: {dist}")
            sound_propagation_speed = SPEED_OF_SOUND + self.rx_pose.pose.position.z/1000 * 17
            delay = dist/sound_propagation_speed
            rospy.loginfo(f"Delay time: {delay}")
            time.sleep(delay)
            tx_loc = Point()
            tx_loc.x = self.rx_pose.pose.position.x
            tx_loc.y = self.rx_pose.pose.position.y
            tx_loc.z = self.rx_pose.pose.position.z
            self.loc_pub.publish(tx_loc)
                    
        except rospy.ServiceException as e:
            rospy.loginfo("Could not find location")

if __name__ == "__main__":
    rospy.init_node("usbl_comm_testing", anonymous=True)

    transponder_id = "1"
    transponder_model = "box1"
    transceiver_id = "168"
    transceiver_model = "sphere1"

    tx1 = Transceiver(transponder_id, transponder_model, transceiver_id, transceiver_model)
    tx1.set_init_pose(0, 0, 0.5)
    rx1 = Transponder(transponder_id, transponder_model, transceiver_id, transceiver_model)
    rx1.set_init_pose(3, 3, 0.5)

    
    while not rospy.is_shutdown():
        time.sleep(5)
        tx1.set_interrogation_mode("common")
        tx1.switch_channel("1")
        tx1.send_ping()
        time.sleep(10)
        rospy.Rate(10).sleep()