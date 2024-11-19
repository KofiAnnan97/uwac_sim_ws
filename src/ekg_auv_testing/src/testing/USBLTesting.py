#!/usr/bin/env python3

import math
import time
import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from rospy_message_converter import message_converter, json_message_converter

from seatrac_pkg.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, bcn_status_array, bcn_status, bcn_remote_gps, head, loc

from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim, VehiclePose, Packet, Payload

INTERROGATION_MODE = ['common', 'individual']

GAZEBO_GET_MODEL_STATE_TOPIC = 'gazebo/get_model_state'
GAZEBO_MODEL_STATES_TOPIC = 'gazebo/model_states'

SPEED_OF_SOUND = 1540                                     # speed of sound underwater

class Transceiver():
    def __init__(self, transponder_id, transceiver_id, model): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.mode = INTERROGATION_MODE[0]
        self.transceiver_model = model
        self.transponder_model = None
        self.tx_pose = bcn_pose()

        self.models = ModelStates()

        self.common_data = None
        self.transponder_location = VehiclePose()
        self.tx_locs = dict()

        self.rate = rospy.Rate(10)

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
        #self.comm_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=10)
        self.comm_pub = rospy.Publisher(COMMON_TOPIC, Packet, queue_size=10)
        self.tx_loc_pub = rospy.Publisher(TRANSPONDER_LOCATION_TOPIC, VehiclePose, queue_size=20)
        #self.resp_pub = rospy.Publisher(RESPONSE_TOPIC, USBLResponseSim, queue_size=1)
        self.resp_pub = rospy.Publisher(RESPONSE_TOPIC, Packet, queue_size=1)

        # ROS Subscribers
        #rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(COMMON_TOPIC, Packet, self.__common_cbk)
        rospy.Subscriber(CHANNEL_TOPIC, String, self.__channel_cbk)
        rospy.Subscriber(MODE_TOPIC, String, self.__mode_cbk)
        rospy.Subscriber(TRANSPONDER_LOCATION_TOPIC, VehiclePose, self.__rx_loc_cbk)
        #rospy.Subscriber(REQUEST_TOPIC, USBLRequestSim, self.__request_cbk)
        rospy.Subscriber(REQUEST_TOPIC, Packet, self.__request_cbk)

    # Callback Functions
    def __rx_loc_cbk(self, data):
        if data is not None:
            self.transponder_location = data
            #print(data)
            if data.vehicle_name not in self.tx_locs.keys():
                self.tx_locs[data.vehicle_name] = data
            else:
                self.tx_locs[data.vehicle_name] = data

            #rospy.loginfo(f"[Transciever {self.transceiver_id}]: Transponder location at ({data.pose.position.x}, {data.pose.position.y}, {data.pose.position.z})")

    def __common_cbk(self, data):
        if data is not None:
            pass

    def __models_cbk(self, data):
        if data is not None:
            self.models = data
            pose = self.get_model_pos(self.transceiver_model)
            self.tx_pose.bid = self.transceiver_id
            q = pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.tx_pose.roll = roll
            self.tx_pose.pitch = pitch
            self.tx_pose.yaw = yaw
            self.tx_pose.x = pose.position.x
            self.tx_pose.y = pose.position.y
            self.tx_pose.z = pose.position.z
            

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
    
    def __request_cbk(self, msg):
        if msg is not None:
            processed = self.process_data(msg)
            if processed.data == "location":
                self.send_ping()
                self.send_command_response(processed)
                
    # Methods
    def process_data(self, msg):
        if msg is not None:
            try:
                msg_type = msg.rosmsg_type
                msg_str = msg.data
                rosmsg = json_message_converter.convert_json_to_ros_message(msg_type, msg_str)
                return rosmsg
            except Exception as e:
                print(e)
                return None
            
    def load_data(self, type, msg):
        if msg is not None:
            msg_json = json_message_converter.convert_ros_message_to_json(msg) 
            packet = Packet()
            packet.rosmsg_type = type
            packet.data = str(msg_json)
            return packet
        else:
            return None

    def send_ping(self):
        loaded = self.load_data('std_msgs/String', String(data='ping'))
        if self.mode == INTERROGATION_MODE[0]:       # Common channel
            self.comm_pub.publish(loaded)
        elif self.mode == INTERROGATION_MODE[1]:     # Inidividual channel
            # rospy.loginfo(f'/USBL/transponder_{self.transponder_id}/ping')
            # rospy.loginfo(msg)
            individual_pub = rospy.Publisher(f'/USBL/transponder_{self.transponder_id}/ping', Packet, queue_size=1)
            individual_pub.publish(loaded)

    def switch_channel(self, channel_id):
        if channel_id != int(self.transponder_id):
            REQUEST_TOPIC = f'/USBL/transponder_{self.transceiver_id}/command_request'
            #self.requests = rospy.Subscriber(REQUEST_TOPIC, USBLRequestSim, self.__request_cbk)
            self.requests = rospy.Subscriber(REQUEST_TOPIC, Packet, self.__request_cbk)
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
        
    def send_command_response(self, msg):
        resp_msg = USBLResponseSim()
        resp_msg.transceiverID = int(self.transceiver_id)
        resp_msg.transceiverModelName = self.transceiver_model
        resp_msg.responseID = int(msg.responseID)

        if msg.transponderModelName in self.tx_locs.keys():
            tx_loc = self.tx_locs[msg.transponderModelName] 
            loc_str = json_message_converter.convert_ros_message_to_json(tx_loc)
            resp_msg.data = loc_str
        else:
            resp_msg.data = f"No location data found for {msg.transponderModelName}."
        loaded = self.load_data('ekg_auv_testing/USBLResponseSim', resp_msg)
        #self.resp_pub.publish(resp_msg)
        self.resp_pub.publish(loaded)

    def send_msg(self, msg):
        loaded = self.load_data('std_msgs/String', String(data=msg))
        if self.mode == INTERROGATION_MODE[0]:
            self.comm_pub.publish(loaded)
        elif self.mode == INTERROGATION_MODE[1]:
            individual_pub = rospy.Publisher(f'/USBL/transponder_{self.transponder_id}/ping', String, queue_size=1)
            individual_pub.publish(loaded)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()



class Transponder():
    def __init__(self, transponder_id, transceiver_id, model_name): 
        self.transponder_id = transponder_id
        self.transceiver_id = transceiver_id
        self.transceiver_model = None
        self.transponder_model = model_name
        self.rx_pose = bcn_pose()

        self.models = ModelStates()

        self.command_resp = USBLResponseSim()

        self.mu = 0.5
        self.sigma = 5

        self.rate = rospy.Rate(10)

        # Topics
        COMMON_TOPIC = '/USBL/common_ping'
        INDIVIDUAL_TOPIC = f'/USBL/transponder_{self.transponder_id}/ping'
        SWITCH_TOPIC = f'/USBL/transponder_{self.transponder_id}/channel_switch'
        RESPONSE_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'
        REQUEST_TOPIC= f'/USBL/transceiver_{self.transceiver_id}/command_request'

        # Gazebo Subscribers
        rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, self.__models_cbk)

        # ROS Publishers
        #self.loc_pub = rospy.Publisher(self.TRANSPONDER_LOCATION_TOPIC, VehiclePose, queue_size=10)
        #self.request_pub = rospy.Publisher(REQUEST_TOPIC, USBLRequestSim, queue_size=1)
        self.request_pub = rospy.Publisher(REQUEST_TOPIC, Packet, queue_size=1)

        # ROS Subscribers
        #rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        #rospy.Subscriber(INDIVIDUAL_TOPIC, String, self.__individual_cbk)
        #rospy.Subscriber(RESPONSE_TOPIC, USBLResponseSim, self.__resp_cbk)
        rospy.Subscriber(COMMON_TOPIC, Packet, self.__common_cbk)
        rospy.Subscriber(INDIVIDUAL_TOPIC, Packet, self.__individual_cbk)
        rospy.Subscriber(RESPONSE_TOPIC, Packet, self.__resp_cbk)
        rospy.Subscriber(SWITCH_TOPIC, String, self.__channel_cbk)

    def __common_cbk(self, msg):
        if msg is not None:
            type = msg.rosmsg_type
            processed = self.process_data(msg)
            if type == 'std_msgs/String':
                if processed.data == 'ping':
                    rospy.loginfo(f'Pinging {self.transponder_id}')
                    self.query_location()

    def __individual_cbk(self, msg):
        if msg is not None:
            type = msg.rosmsg_type
            processed = self.process_data(msg)
            if type == 'std_msgs/String':
                if processed.data == 'ping':
                    # rospy.loginfo(f'Pinging {self.transponder_id}')
                    self.query_location()
            elif type == 'seatrac_pkg/bcn_frame_array':
                for frame in processed.frame:
                    if frame.data == 'ping':
                        self.query_location()

    def __channel_cbk(self, msg):
        if msg is not None:
            self.set_tx_channel(msg.data)

    def __models_cbk(self, data):
        if data is not None:
            self.models = data
            pose = self.get_model_pos(self.transponder_model)
            self.rx_pose.bid = self.transceiver_id
            q = pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.rx_pose.roll = roll
            self.rx_pose.pitch = pitch
            self.rx_pose.yaw = yaw
            self.rx_pose.x = pose.position.x
            self.rx_pose.y = pose.position.y
            self.rx_pose.z = pose.position.z
            

    def __resp_cbk(self, msg):
        if msg is not None:
            #print(data)
            processed = self.process_data(msg)
            self.command_resp = processed
    
    # Methods
    def __dist_between_points(self, start_pose, end_pose):
        x1 = start_pose.x
        y1 = start_pose.y
        z1 = start_pose.z
        x2 = end_pose.x
        y2 = end_pose.y
        z2 = end_pose.z
        dist = 0
        sum_of_squares = math.pow(x1-x2,  2)+ math.pow(y1-y2, 2) + math.pow(z1-z2, 2) 
        dist = math.sqrt(sum_of_squares)
        return dist

    def process_data(self, msg):
        if msg is not None:
            try:
                msg_type = msg.rosmsg_type
                msg_str = msg.data
                rosmsg = json_message_converter.convert_json_to_ros_message(msg_type, msg_str)
                return rosmsg
            except Exception as e:
                print(e)
                return None
            
    def load_data(self, type, msg):
        if msg is not None:
            msg_json = json_message_converter.convert_ros_message_to_json(msg) 
            packet = Packet()
            packet.rosmsg_type = type
            packet.data = str(msg_json)
            return packet
        else:
            return None

    def send_location_request(self):
        REQUEST_TOPIC= f'/USBL/transceiver_{self.transceiver_id}/command_request'
        self.request_pub = rospy.Publisher(REQUEST_TOPIC, Packet, queue_size=1)

        req = USBLRequestSim()
        req.responseID = int(self.transponder_id)
        req.transceiverID = int(self.transceiver_id)
        req.transponderModelName = self.transponder_model
        req.data = "location"
        loaded = self.load_data('ekg_auv_testing/USBLRequestSim', req)
        # print(f"send_location request: {self.transceiver_id}")
        # print(req, "\n-------------------------------------")
        self.request_pub.publish(loaded)

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
            #start_time = rospy.Time.now()
            tx_pose = bcn_pose()
            pose = self.get_model_pos(self.transceiver_model)
            tx_pose.bid = self.transceiver_id
            q = pose.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            tx_pose.roll = roll
            tx_pose.pitch = pitch
            tx_pose.yaw = yaw
            tx_pose.x = pose.position.x
            tx_pose.y = pose.position.y
            tx_pose.z = pose.position.z

            #dist = self.__dist_between_points(self.rx_pose.pose, tx_pose.pose)
            dist = self.__dist_between_points(self.rx_pose, tx_pose)
            # rospy.loginfo(f"Calculated distance: {dist}")
            sound_propagation_speed = SPEED_OF_SOUND + self.rx_pose.z/1000 * 17
            delay = 2*(dist/sound_propagation_speed)*1
            #rospy.loginfo(f"Delay time: {delay}")
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < delay:
                pass
            #time.sleep(delay)
            loc_pub = rospy.Publisher(f'/USBL/transceiver_{self.transceiver_id}/transponder_pose', VehiclePose, queue_size=20)
            loc = VehiclePose()
            noise = np.random.normal(self.mu, self.sigma, 3)
            loc.stamp = rospy.Time.now() # start_time + rospy.Duration(delay)
            loc.vehicle_name = self.transponder_model
            #rospy.loginfo(f"{self.transceiver_id} || Time: {tx_loc.header.stamp.secs}")
            loc.x = self.rx_pose.x + noise[0]
            loc.y = self.rx_pose.y + noise[1]
            loc.z = self.rx_pose.z + noise[2]
            loc_pub.publish(loc)
                    
        except Exception as e:
            print(f"Query Func: {e}")
            # rospy.loginfo("Could not find location")

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

###########
# TESTING #
###########
# Testing Multi-Transponder communciation 
def test_1(tx1):
    tx1.set_interrogation_mode("common")
    tx1.send_ping()
    time.sleep(2)

# Testing Mutli-Transceiver communciation
def test_2(tx1, tx2, rx1):
    time.sleep(2)
    tx1.set_interrogation_mode("individual")
    tx1.switch_channel("1")
    rx1.send_location_request()

    rx1.set_tx_channel("169")
    rx1.transceiver_model = "box2"

    time.sleep(2)
    tx2.set_interrogation_mode("individual")
    tx2.switch_channel("1")
    rx1.send_location_request()

    rx1.set_tx_channel("168")
    rx1.transceiver_model = "box1"

if __name__ == "__main__":
    rospy.init_node("usbl_comm_testing", anonymous=True)
    bcn_type = rospy.get_param('~beacon_type')
    model_name = rospy.get_param('~model_name')
    comm_id = rospy.get_param('~comm_id')
    tx_id = rospy.get_param('~tx_id')
    rx_id = rospy.get_param('~rx_id')
    comm = None

    try:
        if bcn_type == 'X150':
            comm = Transceiver(rx_id, comm_id, model_name)
        elif bcn_type == 'X110':
            comm = Transponder(comm_id, tx_id, model_name)
        comm.run()
    except rospy.ROSInterruptException:
        pass

    """rx_id1 = "1"
    rx_id2 = "2"
    rx_model1 = "sphere1"
    rx_model2 = "sphere2"
    
    tx_id1 = "168"
    tx_id2 = "169"
    tx_model1 = "box1"
    tx_model2 = "box2"

    tx1 = Transceiver(rx_id2, tx_id1, tx_model1)
    tx2 = Transceiver(rx_id1, tx_id2, tx_model2)
    rx1 = Transponder(rx_id1, tx_id1, rx_model1)
    rx2 = Transponder(rx_id2, tx_id1, rx_model2)
    
    rx1.mu = 0.2
    rx1.sigma = 0.07"""
