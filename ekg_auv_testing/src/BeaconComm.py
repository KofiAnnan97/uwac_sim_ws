#!/usr/bin/env python3

import rospy
import math
import time
import json
from sys import argv

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from std_msgs.msg import String
from rospy_message_converter import message_converter, json_message_converter

from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim, Packet
from seatrac_pkg.msg import *
from USBL import Transceiver

SEND_POS_MSG = "location"

class SimpleBeaconNode:
    def __init__(self) -> None:
        self.true_pose = PoseStamped()        
        
        self.model_name = argv[1]
        self.transponder_id = argv[2]
        self.transceiver_id = argv[6]
        
        #self.tx = Transceiver(self.transponder_id, self.transceiver_id, self.model_name)
        
        self.init_pose = PoseStamped()
        
        self.init_pose.pose.position.x = float(argv[3])
        self.init_pose.pose.position.y = float(argv[4])
        self.init_pose.pose.position.z = float(argv[5])
        self.curr_pose = self.init_pose
        self.last_pose = PoseStamped()
        
        self.water_pressure = FluidPressure()
        self.imu = Imu()
        self.gps = NavSatFix()

        self.neighbors = dict()

        self.id = argv[2]

        self.layer_idx = 0
    
    def init_app(self):
        # Location Topics
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose' 
        TRUE_EULER_TOPC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/euler'
        ODOM_TOPIC = f'/{argv[1]}/pose_gt'
        
        # Movement Topics
        MOVE_TOPIC = f'/{argv[1]}/cmd_vel'

        # Common Topics
        COMMON_TOPIC = '/USBL/common_ping'

        # Transciever Topics
        RESPONSE_TOPIC = f"/USBL/transceiver_{self.transceiver_id}/command_response"
        INTERROGATION_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/interrogation_mode'
        CHANNEL_SWITCH_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/channel_switch'
        TARGET_LOC_TOPIC = f'/USBL/transceiver_{self.transceiver_id}/transponder_location'

        # Transponder Topics
        REQUEST_TOPIC = f"/USBL/transponder_manufacturer_{self.transponder_id}/command_request"
        INDIVIDUAL_TOPIC = f'/USBL/transponder_manufacturer_{self.transponder_id}/individual_interrogation_ping'

        # Sensor Topics
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'

        self.rate = rospy.Rate(10)
        self.target_pos = bcn_pose()

        # Publishers
        self.move_pub = rospy.Publisher(MOVE_TOPIC, Twist, queue_size=1)
        self.channel_pub = rospy.Publisher(CHANNEL_SWITCH_TOPIC, String, queue_size=1)
        self.common_pub = rospy.Publisher(COMMON_TOPIC, Packet, queue_size=30)
        self.inter_pub = rospy.Publisher(INTERROGATION_TOPIC, String, queue_size=1)  

        # Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.__odom_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        # rospy.Subscriber(TARGET_LOC_TOPIC, Vector3, self.__target_pos_cbk)
        #rospy.Subscriber(COMMON_TOPIC, String, self.__common_cbk)
        rospy.Subscriber(COMMON_TOPIC, Packet, self.__common_cbk)
        # rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)

    ###################### 
    # Callback Functions #
    ######################

    def __true_pose_cbk(self, data):
        self.true_pose = data
        self.curr_pose = self.true_pose

    def __odom_cbk(self, data):
        pos = data.pose.position
        #rospy.loginfo(f"{argv}: ({pos.x}, {pos.y}, {pos.z})")

    def __target_pos_cbk(self, data):
            if data is not None:
                self.target_pos = data
                """self.target_pos.pose.position.x = data.x
                self.target_pos.pose.position.y = data.y
                self.target_pos.pose.position.z = data.z"""
            else:
                rospy.loginfo(f"{argv[1]} has not detected transponder yet.")

    def __imu_cbk(self, data):
        self.imu = data

    def __pressure_cbk(self, data):
        self.water_pressure = data

    def __gps_cbk(self, data):
        try:        
            self.gps = data
        except:
            pass

    # Check what transceivers that are set to the same transponder are close
    def __common_cbk(self, msg):
        if msg is not None:
            processed = self.process_data(msg)
            #print(processed)
            try:
                if msg.rosmsg_type == 'seatrac_pkg/bcn_pose':
                    self.neighbors[processed.bid] = processed
            except Exception as e:
                print(e)
    
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

    ##############################
    # Movement/Sensors Functions #
    ##############################
    
    def get_depth(self):
        depth = 0
        try:
            standardPressure = 101.325  # data found in glider_hybrid_whoi/glidereckoining/nodes/deadreckoning_estimator.py 
            KPaPerM = 9.80838
            depth = -round((self.water_pressure.fluid_pressure - standardPressure)/KPaPerM, 2)
            return depth
            # rospy.loginfo(f"Depth Estimate: {self.depth_estimate}, Actual Depth: {self.true_pose.pose.position.z}")
        except:
            rospy.loginfo("Depth could not be calculated.")
            return 0
     
     ###########################
     # Communication Functions #
     ###########################  

    def switch_to_target(self, transponder_id):
        msg = String()
        msg.data = "individual"
        self.inter_pub.publish(msg)
        msg.data = transponder_id
        self.channel_pub.publish(msg)

    def switch_to_common(self):
        msg = String()
        msg.data = "common"
        self.inter_pub.publish(msg)

    def send_target_pos(self, transponder_id):
        loc_pub = rospy.Publisher(f"/USBL/transponder_manufacturer_{transponder_id}/ping", Packet, queue_size=20)
        self.switch_to_target(transponder_id, loc_pub)

        if self.target_pos is not None:
            #json_str = json_message_converter.convert_ros_message_to_json(self.target_pos)
            """pos_msg = String()
            pos_obj = json.loads(json_str)
            beacon_id = {"bid":self.id}
            pos_obj.update(beacon_id)
            pos_str = json.dumps(pos_obj)
            pos_msg.data = str(pos_str)"""

            orient = self.imu.orientation
            roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            loc = bcn_pose()
            loc.bid = self.transceiver_id
            loc.stamp = rospy.Time.now()
            loc.roll = roll
            loc.pitch = pitch
            loc.yaw = yaw
            loc.x = self.target_pos.pose.position.x
            loc.y = self.target_pos.pose.position.y
            loc.z = self.target_pos.pose.position.z
            loaded = self.load_data('seatrac_pkg/bcn_pose', loc)
            loc_pub.publish(loaded)

    def send_pose_to_common(self):
        self.switch_to_common()
        self.curr_pose.header.stamp = rospy.Time().now()
        orient = self.imu.orientation
        roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        loc = bcn_pose()
        loc.bid = self.transceiver_id
        loc.roll = roll
        loc.pitch = pitch
        loc.yaw = yaw
        loc.x = self.curr_pose.pose.position.x
        loc.y = self.curr_pose.pose.position.y
        loc.z = self.curr_pose.pose.position.z
        loaded = self.load_data('seatrac_pkg/bcn_pose', loc)
        #print(loaded)
        self.common_pub.publish(loaded)

    #############
    # Main Code #
    ############# 

    def run(self):
        rospy.loginfo(f"{argv[1]} node has started.")
        self.switch_to_common()
        while not rospy.is_shutdown():
            self.send_pose_to_common()
            time.sleep(3)
            self.rate.sleep()    

if __name__ == "__main__":
    rospy.init_node("simple_node", anonymous=True)
    node = SimpleBeaconNode()
    node.init_app()
    try:
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{argv[1]} node has been terminated.")