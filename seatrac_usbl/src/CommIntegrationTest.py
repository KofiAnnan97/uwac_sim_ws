#!/usr/bin/env python3

import numpy as np
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from seatrac_pkg.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose
from ekg_auv_testing.msg import Payload, Packet, VehiclePose, USBLRequestSim, USBLResponseSim
from rospy_message_converter import message_converter, json_message_converter

from MessageTypes import *
from CommHandlerSim import *
from CsvLogger import *

class Test(object):
    # Name          : __init__
    # Description   : class constructor
    def __init__(self):
        # Node initialization and definition of execution rate parameters of node
        self.ctr = 0
        self.comm_logger = 0
        
        rospy.init_node('IntegrationTestNode', anonymous=True) 
        self.rate = rospy.Rate(0.5)
        self.bcn_type = rospy.get_param('~beacon_type') 
        self.rx_id = rospy.get_param('~transponder_id')
        self.tx_id = rospy.get_param('~transceiver_id')

        RX_TOPIC = f'/USBL/transponder_{self.rx_id}/ping'
        RX_POSE_TOPIC = f'/USBL/transceiver_{self.tx_id}/transponder_pose'
        self.bcnTxPub = rospy.Publisher(RX_TOPIC, Packet, queue_size=10)
        rospy.Subscriber(RX_TOPIC, Packet, self.__rxCallback)
        rospy.Subscriber(RX_POSE_TOPIC, VehiclePose, self.__rxPoseCallback)

        self.rx_pose = None
        
    def runner(self):
        # Start ROS main loop
        while not rospy.is_shutdown():
            frame_array = bcn_frame_array()
            frame = bcn_frame()
            if(self.bcn_type == 'X150'):
                
                frame.bid = self.tx_id
                frame.data = 'ping'
            else:
                frame.bid == self.rx_id
                frame.data = str(self.rx_pose)

            frame_array.frame.append(frame)
            loaded = self.load_data('seatrac_pkg/bcn_frame_array',frame_array)
            """packet = Packet()
            packet.rosmsg_type = "seatrac_pkg/bcn_frame"
            packet.data = String(data=frame)"""
            self.bcnTxPub.publish(loaded)
            self.rate.sleep()

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

    def __rxCallback(self,msg):
        if msg is not None:
            #print(msg)
            frame_array = self.process_data(msg)
            #print(frame_array)
            for frame in frame_array.frame:
                rospy.loginfo('__rxCallback %s, %s'%(frame.bid,frame.data))

    def __rxPoseCallback(self, msg):
        if msg is not None:
            self.rx_pose = msg
            rospy.loginfo(f'__rxPoseCallback ({self.rx_pose.pose.position.x},{self.rx_pose.pose.position.y},{self.rx_pose.pose.position.z})')

if __name__ == '__main__':
    Test().runner() 
