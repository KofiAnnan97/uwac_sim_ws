#!/usr/bin/env python
# System Imports
import os
import numpy as np
import math as m
import rospy
import time
from datetime import datetime

# ROS Imports
from seatrac_pkg.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, bcn_status, bcn_status_array, bcn_remote_gps, loc, IverOSI, head

# Uncomment following lines when final integration is done, and remove messages
# from seatrac_usbl package.
# A copy of the loc and IverOSI messages is kept inside seatrac_usbl package in
# order to make development simpler and keep packages isolated.

# from backseat_control.msg import IverOSI
# from aut_cat.msg import loc
# from aut_cat.msg import head

from MessageTypes import *
from CommHandler import *
from CsvLogger import *
from GpsCalculations import *
from AppMsgProtocol import *

# GPS  - /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
# x110 - /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
# x150 - /dev/serial/by-id/usb-FTDI_US232R_FT2L0U9L-if00-port0

class Beacon(object):

    # Name          : __init__
    # Description   : class constructor
    def __init__(self):
        # Time variable to keep track of the schedule table.
        self.start_time = 0
        # Object variable, it will hold the object created to handle the
        # communication and message handlers.
        self.comm_hdlr = 0
        self.msg_hdlr = 0
        self.gps_logger = 0
        self.usbl_logger = 0
        self.pos_logger = 0
        self.ahrs_logger = 0
        # Flag to determine if the connection with the beacon has been stablished and the
        # ID of the beacon has been detected.
        self.connected_flag = False
        # Upon startup we need to know which beacon we are connected to, this variable
        # will hold that information.
        self.local_bid = ''
        # Flag to enable the message dispatcher, its main usage is to prevent
        # data collision, which causes the beacon to get stuck.
        self.enable_dispatcher = True
        # Internal dictionaries to keep track of beacon positions and communication buffers
        self.bcn_list = [(15,'BEACON_ID15'),(14,'BEACON_ID14'),(13,'BEACON_ID13')]
        self.bcn_ctr = 0
        self.bcn_ctr_gps = 0
        self.gps_pos = {'lat':0.0,'long':0.0}
        self.iver_state = {'FYT':0, 'FYT':0, 'FYB':0,'FPR':0, 'FPL':0, 'MS':0, 'Mode':0, 'NextWP': 0, 'latitude':0.0, 'longitude':0.0, 'speed': 0.0, 'distanceToNext':0.0, 'error':0, 'altitude':0.0, 'parkTime':0, 'magDeclination':0.0, 'missionTime':0.0, 'trueHeading':0.0, 'depth':0.0, 'battery':0.0}
        self.fit_err = 0.0

        self.bcn_pose_dict =    {0:{'bid':'BEACON_ID0', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                1:{'bid':'BEACON_ID1', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                2:{'bid':'BEACON_ID2', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                3:{'bid':'BEACON_ID3', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                4:{'bid':'BEACON_ID4', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                5:{'bid':'BEACON_ID5', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                6:{'bid':'BEACON_ID6', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                7:{'bid':'BEACON_ID7', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                8:{'bid':'BEACON_ID8', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                9:{'bid':'BEACON_ID9', 'roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0,  'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                10:{'bid':'BEACON_ID10','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                11:{'bid':'BEACON_ID11','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                12:{'bid':'BEACON_ID12','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                13:{'bid':'BEACON_ID13','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                14:{'bid':'BEACON_ID14','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 },
                                15:{'bid':'BEACON_ID15','roll':-181.0, 'yaw':-1.0, 'pitch':-91.0, 'x':0.0, 'y':0.0, 'z':0.0, 'azim': 0.0, 'elev':0.0, 'range':0.0, 'ext_head':0.0, 'supp_volt':-1.0, 'temp':0.0, 'press':0.0, 'vos':0.0, 'c_acc_x':0.0, 'c_acc_y':0.0, 'c_acc_z':0.0, 'c_gyro_x':0.0, 'c_gyro_y':0.0, 'c_gyro_z':0.0, 'c_mag_x':0.0, 'c_mag_y':0.0, 'c_mag_z':0.0, 'r_acc_x':0.0, 'r_acc_y':0.0, 'r_acc_z':0.0, 'r_gyro_x':0.0, 'r_gyro_y':0.0, 'r_gyro_z':0.0, 'r_mag_x':0.0, 'r_mag_y':0.0, 'r_mag_z':0.0 }}

        self.bcn_rxbuff_dict = {0:{'bid':'BEACON_ID0', 'data':''},
                                1:{'bid':'BEACON_ID1', 'data':''},
                                2:{'bid':'BEACON_ID2', 'data':''},
                                3:{'bid':'BEACON_ID3', 'data':''},
                                4:{'bid':'BEACON_ID4', 'data':''},
                                5:{'bid':'BEACON_ID5', 'data':''},
                                6:{'bid':'BEACON_ID6', 'data':''},
                                7:{'bid':'BEACON_ID7', 'data':''},
                                8:{'bid':'BEACON_ID8', 'data':''},
                                9:{'bid':'BEACON_ID9', 'data':''},
                                10:{'bid':'BEACON_ID10', 'data':''},
                                11:{'bid':'BEACON_ID11', 'data':''},
                                12:{'bid':'BEACON_ID12', 'data':''},
                                13:{'bid':'BEACON_ID13', 'data':''},
                                14:{'bid':'BEACON_ID14', 'data':''},
                                15:{'bid':'BEACON_ID15', 'data':''}}

        self.bcn_txbuff_dict = {0:{'bid':'BEACON_ID0', 'data':''},
                                1:{'bid':'BEACON_ID1', 'data':''},
                                2:{'bid':'BEACON_ID2', 'data':''},
                                3:{'bid':'BEACON_ID3', 'data':''},
                                4:{'bid':'BEACON_ID4', 'data':''},
                                5:{'bid':'BEACON_ID5', 'data':''},
                                6:{'bid':'BEACON_ID6', 'data':''},
                                7:{'bid':'BEACON_ID7', 'data':''},
                                8:{'bid':'BEACON_ID8', 'data':''},
                                9:{'bid':'BEACON_ID9', 'data':''},
                                10:{'bid':'BEACON_ID10', 'data':''},
                                11:{'bid':'BEACON_ID11', 'data':''},
                                12:{'bid':'BEACON_ID12', 'data':''},
                                13:{'bid':'BEACON_ID13', 'data':''},
                                14:{'bid':'BEACON_ID14', 'data':''},
                                15:{'bid':'BEACON_ID15', 'data':''}}
        # Setup Publishers
        self.bcnRxPub = rospy.Publisher('beacon_rx_buffer', bcn_frame_array, queue_size=10)
        self.bcnPosePub = rospy.Publisher('beacon_pose', bcn_pose_array, queue_size=10)
        self.bcnStatPub = rospy.Publisher('beacon_status', bcn_status_array, queue_size=10)
        self.bcnRemoteGpsPub = rospy.Publisher('bcn_remote_gps', bcn_remote_gps, queue_size=10)
        # Subscribe to topics
        rospy.Subscriber('beacon_tx_buffer',bcn_frame_array, self.__txCallback)
        rospy.Subscriber('beacon_gps_data',loc, self.__gpsCallback, queue_size=1)
        rospy.Subscriber('gps',loc, self.__gpsCallback, queue_size=1)
        rospy.Subscriber('iver_state', IverOSI, self.__iverStateCallback, queue_size=1)
        rospy.Subscriber('heading', head, self.__headingCallback, queue_size=1)
        # Node initialization and definition of execution rate parameters of node
        rospy.init_node('SeaTracBeacon', anonymous=True)
        self.rate = rospy.Rate(10)
        self.bcn_type = rospy.get_param('~beacon_type')
        self.wrk_mode = rospy.get_param('~working_mode')
        self.comm_pt_no = rospy.get_param('~comm_port')
        # Initialization of BcnLogger define the name of the directory to be
        # created
        # Decide the name of the loggers based on the beacon where the node is running
        currentFolder = os.path.dirname(os.path.realpath(__file__))
        if(self.bcn_type == 'X150'):
            self.gps_logger  = CsvLogger(path = currentFolder + '/log/x150/' + self.comm_pt_no + '/gps',   loggername = 'x150_gps',  header='lat,long',format ='%f, %f')
            self.usbl_logger = CsvLogger(path = currentFolder + '/log/x150/' + self.comm_pt_no + '/usbl',  loggername = 'x150_usbl', header='sig_peak,thresh,cross_point,cross_mag,detect,length,xcor_data_af,channels,rssi,bslns,phase_ang,azim,elev,fit_error',format ='%f,%f,%d,%f,%d,%d,%s,%d,%s,%d,%s,%f,%f,%f')
            self.pos_logger  = CsvLogger(path = currentFolder + '/log/x150/' + self.comm_pt_no + '/master',loggername = 'x150_pose', header='bid,x,y,z,azim,elev,range,pitch,roll,yaw,ext_head,volt,temp,lat,long,est_lat,est_long,fit_error', format ='%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f')
            self.ahrs_logger = CsvLogger(path = currentFolder + '/log/x150/' + self.comm_pt_no + '/ahrs',  loggername = 'x150_ahrs', header='bid,r_acc_x,r_acc_y,r_acc_z,r_gyro_x,r_gyro_y,r_gyro_z,r_mag_x,r_mag_y,r_mag_z,c_acc_x,c_acc_y,c_acc_z,c_gyro_x,c_gyro_y,c_gyro_z,c_mag_x,c_mag_y,c_mag_z', format ='%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f')
        else:
            self.gps_logger  = CsvLogger(path = currentFolder + '/log/x110/' + self.comm_pt_no + '/gps',   loggername = 'x110_gps',  header='lat,long',format ='%f, %f')
            self.pos_logger  = CsvLogger(path = currentFolder + '/log/x110/' + self.comm_pt_no + '/slave', loggername = 'x110_pose', header='bid,x,y,z,azim,elev,range,pitch,roll,yaw,ext_head,volt,temp,lat,long,est_lat,est_long,fit_error', format ='%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f')
            self.iver_logger = CsvLogger(path = currentFolder + '/log/x110/' + self.comm_pt_no + '/iver',  loggername = 'x110_iver', header='FYT,FYB,FPR,FPL,MS,Mode,NextWP,lat,long,speed,dist2Next,error,alt,prkTime,magDecl,missionTime,trueHeading,depth,battery', format ='%d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %d, %f, %d, %f, %f, %f, %f, %f')
            self.ahrs_logger = CsvLogger(path = currentFolder + '/log/x110/' + self.comm_pt_no + '/ahrs',  loggername = 'x110_ahrs', header='bid,r_acc_x,r_acc_y,r_acc_z,r_gyro_x,r_gyro_y,r_gyro_z,r_mag_x,r_mag_y,r_mag_z,c_acc_x,c_acc_y,c_acc_z,c_gyro_x,c_gyro_y,c_gyro_z,c_mag_x,c_mag_y,c_mag_z', format ='%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f')
        self.comm_pt_no = int(self.comm_pt_no)
        # Configure the beacon port to be used depending on the parameters passed
        # in the launch file
        if(self.bcn_type == 'X150'):
            # X150 - port name needs to be updated in case a different usb-serial adapter is used
            self.bcn_com_port = ['/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0',
                                     '/dev/serial/by-id/usb-FTDI_US232R_FT2L0U9L-if00-port0']
        else:
            # X110 - port name needs to be updated in case a different usb-serial adapter is used
            if os.environ['LOGNAME'] == 'iver':
                rospy.loginfo('===============')
                rospy.loginfo('IVER detected')
                rospy.loginfo('===============')
                self.bcn_com_port = ['/dev/ttyUSB9']
            else:
                self.bcn_com_port = ['/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0',
                                     '/dev/serial/by-id/usb-FTDI_US232R_FT2L0U9L-if00-port0']


    '''
    ================================================================================
    Public functions
    ================================================================================
    '''
    # Name          : sendNavQuery
    # Description   : Sends CID_NAV_QUERY_SEND command to all beacons defined in
    #                 self.bcn_list array.
    # return        : void
    def sendNavQuery(self):
        navQryHdlr = CID_NAV_QUERY_SEND()
        # Send a request for each of the beacons in the list
        for i,bcn in self.bcn_list:
            navQryHdlr.dest_id_e.Bid = bcn
            txframe = navQryHdlr.encode_frame()
            #rospy.loginfo('[scanRemoteBeacons]' + ' - ' + str(navQryHdlr.dest_id_e.Bid))
            # send the created frame to the communication handler.
            self.comm_hdlr.sendMessage(txframe)

    # Name          : pushMsgToQueue
    # Description   : Pushes the data into the local beacon queue for the
    #                 corresponding beacon. This method is used as the interface
    #                 to send data back to the master from the slave beacons.
    # return        : void
    def pushMsgToQueue(self, bcn_id, msg):
        navQueueSetHdlr = CID_NAV_QUEUE_SET(beacon_id = bcn_id, packet_data = msg)
        # Encode the message according to the input parameters
        txframe = navQueueSetHdlr.encode_frame()
        # rospy.loginfo('[scanRemoteBeacons]' + ' - ' + str(scanMsg.dest_id_e.Bid))
        # send the created frame to the communication handler.
        self.comm_hdlr.sendMessage(txframe)

    # Name          : getBeaconStatus
    # Description   : Sends CID_STATUS command to the local beacon
    # return        : void
    def getBeaconStatus(self):
        statusHdlr = CID_STATUS()
        txframe = statusHdlr.encode_frame()
        #rospy.loginfo('[scanLocalBeacon]' + ' - ' + txframe)
        self.comm_hdlr.sendMessage(txframe)

    # Name          : getLocalConfiguration
    # Description   : Function to send a request to get the settings from
    #                 working RAM of the local beacon.
    # return        : void
    def getLocalConfiguration(self):
        getConfigHdlr = CID_SETTINGS_GET()
        txframe = getConfigHdlr.encode_frame()
        rospy.loginfo('[getLocalConfiguration]' + ' - ' + txframe)
        self.comm_hdlr.sendMessage(txframe)

    # Name          : rebootBeacon
    # Description   : Function to trigger a software reboot in the local beacon.
    # return        : void
    def rebootBeacon(self):
        rebootHdlr = CID_SYS_REBOOT()
        txframe = rebootHdlr.encode_frame()
        rospy.loginfo('[rebootBeacon] Starting reboot ' + txframe)
        self.comm_hdlr.sendMessage(txframe)

    # Name          : dispatchMessages
    # Description   : takes all messages in local array and sends the
    #                 information to the corresponding beacon using a
    #                 different API depending on the type of beacon
    #                 (x110 or x150).
    # return        : void
    def dispatchMessages(self):
        # Go through all the entries in the local array looking for pending
        # messages to send.
        i,bid = self.bcn_list[self.bcn_ctr]
        #if(self.bcn_txbuff_dict[i]['data'] != ''):
        if(len(self.bcn_txbuff_dict[i]['data']) < 29):
            bid = self.bcn_txbuff_dict[i]['bid']
            msg = self.bcn_txbuff_dict[i]['data']
            # Use a different command class depending on the beacon type
            if(self.bcn_type == 'X150'):
                if(self.enable_dispatcher == True):
                    navQryHdlr = CID_NAV_QUERY_SEND(beacon_id = bid, packet_data = msg)
                    rospy.loginfo('dispatchMessages x150')
                    rospy.loginfo('sending data to %s %s'%(bid,msg))
                    txframe = navQryHdlr.encode_frame()
                    # Make sure to trigger transmission only if comm handler is up and running
                    if(self.comm_hdlr != 0):
                        # Send the message and clear the output buffer
                        #rospy.loginfo('dispatchMessages')
                        self.comm_hdlr.sendMessage(txframe)
                        self.bcn_txbuff_dict[i]['data'] = ''
                        self.enable_dispatcher = False
                        # Limit the beacon counter and reset it when it reaches the max no of beacons
                        # in the bcn_list
                        self.bcn_ctr += 1
                        if self.bcn_ctr >= len(self.bcn_list):
                            self.bcn_ctr = 0
            else:
                #datQueueSet = CID_DAT_QUEUE_SET(beacon_id = bid, packet_data = msg)
                bid = self.bcn_txbuff_dict[1]['bid']  # todo: hardcoded line, improve implementation for slave side
                msg = self.bcn_txbuff_dict[1]['data'] # todo: hardcoded line, improve implementation for slave side
                navQryHdlr = CID_NAV_QUEUE_SET(beacon_id = bid, packet_data = msg)
                rospy.loginfo('dispatchMessages x110')
                rospy.loginfo('sending data to %s %s'%(bid,msg))
                # Encode the frame according to the input parameters
                txframe = navQryHdlr.encode_frame()
                # Make sure to trigger transmission only if comm handler is up and running
                if(self.comm_hdlr != 0):
                    # Send the message and clear the output buffer
                    #rospy.loginfo('dispatchMessages')
                    self.comm_hdlr.sendMessage(txframe)
                    self.bcn_txbuff_dict[1]['data'] = ''

    # Name          : pingAllBeacons
    # Description   : Pings all the beacons in the network
    # return        : void
    def pingAllBeacons(self):
        if(self.enable_dispatcher == True):
            self.enable_dispatcher = False
            i,bid = self.bcn_list[self.bcn_ctr]
            pingHdlr = CID_PING_SEND( beacon_id = bid, msgtype = 'MSG_REQX')
            txframe = pingHdlr.encode_frame()
            if(self.comm_hdlr != 0):
                rospy.loginfo('pingAllBeacons')
                self.comm_hdlr.sendMessage(txframe)
                self.bcn_ctr += 1
                if self.bcn_ctr >= len(self.bcn_list):
                    self.bcn_ctr = 0

    # Name          : pingRoughie
    # Description   : Pings all the beacons in the network
    # return        : void
    def pingRoughie(self):
        if(self.enable_dispatcher == True):
            self.enable_dispatcher = False
            i,bid = (15,'BEACON_ID15')
            pingHdlr = CID_PING_SEND( beacon_id = bid, msgtype = 'MSG_REQX')
            txframe = pingHdlr.encode_frame()
            if(self.comm_hdlr != 0):
                rospy.loginfo('pingRoughie')
                self.comm_hdlr.sendMessage(txframe)

    # Name          : pingTgt1
    # Description   : Pings all the beacons in the network
    # return        : void
    def pingTgt1(self):
        if(self.enable_dispatcher == True):
            self.enable_dispatcher = False
            i,bid = (14,'BEACON_ID14')
            pingHdlr = CID_PING_SEND( beacon_id = bid, msgtype = 'MSG_REQX')
            txframe = pingHdlr.encode_frame()
            if(self.comm_hdlr != 0):
                rospy.loginfo('pingTgt1')
                self.comm_hdlr.sendMessage(txframe)
    
    # Name          : pingTgt2
    # Description   : Pings all the beacons in the network
    # return        : void
    def pingTgt2(self):
        if(self.enable_dispatcher == True):
            self.enable_dispatcher = False
            i,bid = (13,'BEACON_ID13')
            pingHdlr = CID_PING_SEND( beacon_id = bid, msgtype = 'MSG_REQX')
            txframe = pingHdlr.encode_frame()
            if(self.comm_hdlr != 0):
                rospy.loginfo('pingTgt2')
                self.comm_hdlr.sendMessage(txframe)


    # Name          : sendLocalPosToAllBeacons
    # Description   : Send the local gps position to the rest of the beacons in the network
    # return        : void
    def sendLocalPosToAllBeacons(self):
        i,bid = self.bcn_list[self.bcn_ctr_gps]
        msg = AppMsg_GpsHead([self.gps_pos['long'],self.gps_pos['lat'],self.bcn_pose_dict[1]['ext_head']]).encode_message()
        cidDataSend = CID_DAT_SEND(beacon_id = bid, msgtype = 'MSG_REQX', packet_data = msg)
        txframe = cidDataSend.encode_frame()
        if(self.comm_hdlr != 0):
            rospy.loginfo('sendLocalPosToAllBeacons')
            self.comm_hdlr.sendMessage(txframe)
            self.bcn_ctr_gps += 1
            if(self.bcn_ctr_gps >= len(self.bcn_list)):
                self.bcn_ctr_gps = 0

    # Name          : datSendAllBeacons
    # Description   : Send the content in TX buffer to the remote beacons
    # return        : void
    def datSendAllBeacons(self):
        for i,bid in self.bcn_list:
            self.bcn_txbuff_dict[i]['data']
            cidDataSend = CID_DAT_SEND(beacon_id = bid, msgtype = 'MSG_REQX', packet_data = self.bcn_txbuff_dict[i]['data'])
            txframe = cidDataSend.encode_frame()
            if(self.comm_hdlr != 0):
                rospy.loginfo('datSendAllBeacons')
                self.comm_hdlr.sendMessage(txframe)
                self.bcn_txbuff_dict[i]['data'] = ''
    '''
    ================================================================================
    Private functions
    ================================================================================
    '''
    # define a function to handle other nodes' data published into  txBuffer
    # topic
    def __txCallback(self,data):
        # Go through array received
        for msg in data.frame:
            i = BID_E(bid = msg.bid).encode_data_dec()
            self.bcn_txbuff_dict[i]['data'] = str(msg.data)
        #rospy.loginfo('__txCallback %s'%(msg.data))

    # Name          : __gpsCallback
    # Description   : Function handler triggered upon the reception of gps data.
    # return        : void
    def __gpsCallback(self,data):
        self.gps_pos['lat'] = data.lat
        self.gps_pos['long'] = data.long
        if(self.gps_logger != 0):
            self.gps_logger.logData((self.gps_pos['lat'],self.gps_pos['long']))

        #rospy.loginfo('__gpsCallback lat:%f, long:%f'%(data.lat,data.long))

    # Name          : __iverStateCallback
    # Description   : Function handler triggered upon the reception of iver
    #                 state data.
    # return        : void
    def __iverStateCallback(self,data):
        if self.connected_flag == True:
            self.iver_state['FYT'] = data.FYT
            self.iver_state['FYB'] = data.FYB
            self.iver_state['FPR'] = data.FPR
            self.iver_state['FPL'] = data.FPL
            self.iver_state['MS'] = data.MS
            self.iver_state['Mode'] = data.Mode
            self.iver_state['NextWP'] = data.NextWP
            self.iver_state['latitude'] = data.latitude
            self.iver_state['longitude'] = data.longitude
            self.iver_state['speed'] = data.speed
            self.iver_state['distanceToNext'] =  data.distanceToNext
            self.iver_state['error'] =  data.error
            self.iver_state['altitude'] =  data.altitude
            self.iver_state['parkTime'] =  data.parkTime
            self.iver_state['magDeclination'] =  data.magDeclination
            self.iver_state['missionTime'] =  data.missionTime
            self.iver_state['trueHeading'] =  data.trueHeading
            self.iver_state['depth'] =  data.depth
            self.iver_state['battery'] =  data.battery
            if(self.iver_logger != 0):
                self.iver_logger.logData((self.iver_state['FYT'],
                                          self.iver_state['FYB'],
                                          self.iver_state['FPR'],
                                          self.iver_state['FPL'],
                                          self.iver_state['MS'],
                                          self.iver_state['Mode'],
                                          self.iver_state['NextWP'],
                                          self.iver_state['latitude'],
                                          self.iver_state['longitude'],
                                          self.iver_state['speed'],
                                          self.iver_state['distanceToNext'],
                                          self.iver_state['error'],
                                          self.iver_state['altitude'],
                                          self.iver_state['parkTime'],
                                          self.iver_state['magDeclination'],
                                          self.iver_state['missionTime'],
                                          self.iver_state['trueHeading'],
                                          self.iver_state['depth'],
                                          self.iver_state['battery']))
            rospy.loginfo('__iverStateCallback speed = %f, heading = %f'%(self.iver_state['speed'],self.iver_state['trueHeading']))

    # Name          : __headingCallback
    # Description   : heading callback triggered upon the reception of heading message from the
    #                 autonomous boat.
    # return        : void
    def __headingCallback(self,data):
        self.bcn_pose_dict[1]['ext_head'] = data.heading
        #rospy.loginfo('__headingCallback %f'%(self.bcn_pose_dict[1]['ext_head']))

    # Name          : __publishRxBuffer
    # Description   : Publishes into ROS received data from the remote beacons.
    # return        : void
    def __publishRxBuffer(self):
        frame_array = bcn_frame_array()
        published_msgs = []
        for i in range(len(self.bcn_rxbuff_dict)):
            if((self.bcn_rxbuff_dict[i]['data'] != '') and (self.bcn_rxbuff_dict[i]['data'])):
                # Create an object per frame so the array points to different objects
                frame = bcn_frame()
                # Populate the frame with locally maintained data
                frame.bid = self.bcn_rxbuff_dict[i]['bid']
                frame.data = self.bcn_rxbuff_dict[i]['data']
                #rospy.loginfo(frame.data)
                frame_array.frame.append(frame)
                published_msgs.append(i)
        # Publish data into ROS
        self.bcnRxPub.publish(frame_array)
        # Clear after message is published
        for i in published_msgs:
            self.bcn_rxbuff_dict[i]['data'] = ''
        #rospy.loginfo('__publishRxBuffer')

    # Name          : __publishBeaconPoseArray
    # Description   : publishes the local array of pose data to the BeaconPose topic
    #                 so it becomes available to other nodes.
    # return        : void
    def __publishBeaconPoseArray(self):
        pose_array = bcn_pose_array()
        for i in range(len(self.bcn_pose_dict)):
            if(self.bcn_pose_dict[i]['roll'] != -181.0):
                pose = bcn_pose()
                pose.bid = self.bcn_pose_dict[i]['bid']
                pose.roll = self.bcn_pose_dict[i]['roll']
                pose.pitch = self.bcn_pose_dict[i]['pitch']
                pose.yaw = self.bcn_pose_dict[i]['yaw']
                pose.x = self.bcn_pose_dict[i]['x']
                pose.y = self.bcn_pose_dict[i]['y']
                pose.z = self.bcn_pose_dict[i]['z']
                pose_array.pose.append(pose)
        self.bcnPosePub.publish(pose_array)
        #rospy.loginfo('__publishBeaconPoseArray')

    # Name          : __publishBeaconStatusArray
    # Description   : publishes the local array of status data to the BeaconStatus topic
    #                 so it becomes available to other nodes.
    # return        : void
    def __publishBeaconStatusArray(self):
        stat_array = bcn_status_array()
        for i in range(len(self.bcn_pose_dict)):
            if(self.bcn_pose_dict[i]['supp_volt'] != -1.0):
                status = bcn_status()
                status.bid = self.bcn_pose_dict[i]['bid']
                status.supply_voltage = self.bcn_pose_dict[i]['supp_volt']
                status.temperature = self.bcn_pose_dict[i]['temp']
                status.pressure = self.bcn_pose_dict[i]['press']
                status.vel_of_sound = self.bcn_pose_dict[i]['vos']
                status.comp_acc_x = self.bcn_pose_dict[i]['c_acc_x']
                status.comp_acc_y = self.bcn_pose_dict[i]['c_acc_y']
                status.comp_acc_z = self.bcn_pose_dict[i]['c_acc_z']
                status.comp_gyro_x = self.bcn_pose_dict[i]['c_gyro_x']
                status.comp_gyro_y = self.bcn_pose_dict[i]['c_gyro_y']
                status.comp_gyro_z = self.bcn_pose_dict[i]['c_gyro_z']
                status.comp_mag_x = self.bcn_pose_dict[i]['c_mag_x']
                status.comp_mag_y = self.bcn_pose_dict[i]['c_mag_y']
                status.comp_mag_z = self.bcn_pose_dict[i]['c_mag_z']
                status.raw_acc_x =  self.bcn_pose_dict[i]['r_acc_x']
                status.raw_acc_y =  self.bcn_pose_dict[i]['r_acc_y']
                status.raw_acc_z =  self.bcn_pose_dict[i]['r_acc_z']
                status.raw_gyro_x =  self.bcn_pose_dict[i]['r_gyro_x']
                status.raw_gyro_y =  self.bcn_pose_dict[i]['r_gyro_y']
                status.raw_gyro_z =  self.bcn_pose_dict[i]['r_gyro_z']
                status.raw_mag_x = self.bcn_pose_dict[i]['r_mag_x']
                status.raw_mag_y = self.bcn_pose_dict[i]['r_mag_y']
                status.raw_mag_z = self.bcn_pose_dict[i]['r_mag_z']
                stat_array.status.append(status)
                if(self.ahrs_logger != 0):
                    self.ahrs_logger.logData((i, status.comp_acc_x, status.comp_acc_y, status.comp_acc_z,
                                              status.comp_gyro_x, status.comp_gyro_y, status.comp_gyro_z,
                                              status.comp_mag_x, status.comp_mag_y, status.comp_mag_z,
                                              status.raw_acc_x, status.raw_acc_y, status.raw_acc_z,
                                              status.raw_gyro_x, status.raw_gyro_y, status.raw_gyro_z,
                                              status.raw_mag_x, status.raw_mag_y, status.raw_mag_z))
        self.bcnStatPub.publish(stat_array)
        #rospy.loginfo('__publishBeaconStatusArray')

    # Name          : __logPoseData
    # Description   : Push local variables into the log
    # return        : void
    def __logPoseData(self,bid):
        gpsh = GpsCalculations()
        if (bid == 0):
            for i,bcn in self.bcn_list:
                if(self.bcn_pose_dict[i]['supp_volt'] != -1.0):
                    x = self.bcn_pose_dict[i]['x']
                    y = self.bcn_pose_dict[i]['y']
                    z = self.bcn_pose_dict[i]['z']
                    az = self.bcn_pose_dict[i]['azim']
                    el = self.bcn_pose_dict[i]['elev']
                    rg = self.bcn_pose_dict[i]['range']
                    fterr = self.fit_err
                    pitch = self.bcn_pose_dict[i]['pitch']
                    roll = self.bcn_pose_dict[i]['roll']
                    yaw = self.bcn_pose_dict[i]['yaw']
                    ext_heading = self.bcn_pose_dict[i]['ext_head']
                    volt = self.bcn_pose_dict[i]['supp_volt']
                    temp = self.bcn_pose_dict[i]['temp']
                    lat = self.gps_pos['lat']
                    long = self.gps_pos['long']
                    #rospy.loginfo('logPoseData bid: %d'%(i))
                    #rospy.loginfo('x:%f,y:%f'%(x,y))
                    #rospy.loginfo('ang_ned:%f'%(np.degrees(gpsh.ang_ned(m.atan2(y,x)))))
                    #rospy.loginfo('dist:%f'%(m.sqrt(x*x+y*y)))
                    [est_lat,est_long] = gpsh.destination_point(lat,long,m.sqrt(x*x+y*y),np.degrees(gpsh.ang_ned(m.atan2(y,x))))
                    if(self.pos_logger != 0):
                        self.pos_logger.logData((i,x,y,z,az,el,rg,pitch,roll,yaw,ext_heading,volt,temp,lat,long,est_lat,est_long,fterr))
        else:
            x = self.bcn_pose_dict[bid]['x']
            y = self.bcn_pose_dict[bid]['y']
            z = self.bcn_pose_dict[bid]['z']
            az = self.bcn_pose_dict[bid]['azim']
            el = self.bcn_pose_dict[bid]['elev']
            rg = self.bcn_pose_dict[bid]['range']
            fterr = 0.0
            pitch = self.bcn_pose_dict[bid]['pitch']
            roll = self.bcn_pose_dict[bid]['roll']
            yaw = self.bcn_pose_dict[bid]['yaw']
            ext_heading = self.bcn_pose_dict[bid]['ext_head']
            volt = self.bcn_pose_dict[bid]['supp_volt']
            temp = self.bcn_pose_dict[bid]['temp']
            lat = self.gps_pos['lat']
            long = self.gps_pos['long']
            est_lat = 0.0
            est_long = 0.0
            if(self.pos_logger != 0):
                self.pos_logger.logData((bid,x,y,z,az,el,rg,pitch,roll,yaw,ext_heading,volt,temp,lat,long,est_lat,est_long,fterr))
    def __TerminateNode(self, msg):
        # Finish the execution nicely in case there is no RS232 communication
        rospy.loginfo(msg)
        nodes = os.popen("rosnode list").readlines()
        for node in nodes:
            if 'TestNode' in node:
                os.system("rosnode kill /TestNode")
            if 'GpsTracker' in node:
                os.system("rosnode kill /GpsTracker")
        quit()

    '''
    ================================================================================
    Private functions (Serial Communication Callbacks)
    ================================================================================
    '''
    '''
    ========================================
    Settings Messages
    ========================================
    '''
    # Name          : __CidSettingsGetHandler
    # Description   :
    # return        : void
    def __CidSettingsGetHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        settingsGetHdlr =  CID_SETTINGS_GET()
        # Decode the incomming message
        settingsGetHdlr.decode_response(in_raw_message)
        self.local_bid = settingsGetHdlr.resp_msg_settings_t.xcvr_beacon_id_e.Bid
        #rospy.loginfo('__CidSettingsGetHandler local bid is %s'%(self.local_bid))
    '''
    ========================================
    Status Messages
    ========================================
    '''
    # Name          : __CidStatusHandler
    # Description   :
    # return        : void
    def __CidStatusHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        statusHdlr =  CID_STATUS()
        bidHdlr = BID_E()
        # Decode the incomming message
        statusHdlr.decode_response(in_raw_message)
        bidHdlr.Bid = self.local_bid
        bid = bidHdlr.encode_data_dec()
        # Move decoded data into the local array
        self.bcn_pose_dict[bid]['yaw']       = float(statusHdlr.resp_msg_att_yaw_i16/10)
        self.bcn_pose_dict[bid]['roll']      = float(statusHdlr.resp_msg_att_roll_i16/10)
        self.bcn_pose_dict[bid]['pitch']     = float(statusHdlr.resp_msg_att_pitch_i16/10)
        self.bcn_pose_dict[bid]['x']         = 0
        self.bcn_pose_dict[bid]['y']         = 0
        self.bcn_pose_dict[bid]['z']         = float(statusHdlr.resp_msg_env_depth_i32/10.0)
        self.bcn_pose_dict[bid]['azim']      = 0
        self.bcn_pose_dict[bid]['elev']      = 0
        self.bcn_pose_dict[bid]['range']     = 0
        self.bcn_pose_dict[bid]['supp_volt'] = float(statusHdlr.resp_msg_env_supply_u16/1000)
        self.bcn_pose_dict[bid]['temp']      = float(statusHdlr.resp_msg_env_temp_i16/10)
        self.bcn_pose_dict[bid]['press']     = statusHdlr.resp_msg_env_pressure_i32
        self.bcn_pose_dict[bid]['vos']       = float(statusHdlr.resp_msg_env_vos_u16/10)
        self.bcn_pose_dict[bid]['r_acc_x']   = statusHdlr.resp_msg_ahrs_raw_acc_x_i16
        self.bcn_pose_dict[bid]['r_acc_y']   = statusHdlr.resp_msg_ahrs_raw_acc_y_i16
        self.bcn_pose_dict[bid]['r_acc_z']   = statusHdlr.resp_msg_ahrs_raw_acc_z_i16
        self.bcn_pose_dict[bid]['r_gyro_x']  = statusHdlr.resp_msg_ahrs_raw_gyro_x_i16
        self.bcn_pose_dict[bid]['r_gyro_y']  = statusHdlr.resp_msg_ahrs_raw_gyro_y_i16
        self.bcn_pose_dict[bid]['r_gyro_z']  = statusHdlr.resp_msg_ahrs_raw_gyro_z_i16
        self.bcn_pose_dict[bid]['r_mag_x']   = statusHdlr.resp_msg_ahrs_raw_mag_x_i16
        self.bcn_pose_dict[bid]['r_mag_y']   = statusHdlr.resp_msg_ahrs_raw_mag_y_i16
        self.bcn_pose_dict[bid]['r_mag_z']   = statusHdlr.resp_msg_ahrs_raw_mag_z_i16
        self.bcn_pose_dict[bid]['c_acc_x']   = statusHdlr.resp_msg_ahrs_comp_acc_x_f
        self.bcn_pose_dict[bid]['c_acc_y']   = statusHdlr.resp_msg_ahrs_comp_acc_y_f
        self.bcn_pose_dict[bid]['c_acc_z']   = statusHdlr.resp_msg_ahrs_comp_acc_z_f
        self.bcn_pose_dict[bid]['c_gyro_x']  = statusHdlr.resp_msg_ahrs_comp_gyro_x_f
        self.bcn_pose_dict[bid]['c_gyro_y']  = statusHdlr.resp_msg_ahrs_comp_gyro_y_f
        self.bcn_pose_dict[bid]['c_gyro_z']  = statusHdlr.resp_msg_ahrs_comp_gyro_z_f
        self.bcn_pose_dict[bid]['c_mag_x']   = statusHdlr.resp_msg_ahrs_comp_mag_x_f
        self.bcn_pose_dict[bid]['c_mag_y']   = statusHdlr.resp_msg_ahrs_comp_mag_y_f
        self.bcn_pose_dict[bid]['c_mag_z']   = statusHdlr.resp_msg_ahrs_comp_mag_z_f

        #rospy.loginfo('__logPoseData from __CidStatusHandler')
        self.__logPoseData(bid)
        # Publish the data to ROS, so the rest of the node can read the received
        # position of the beacons
        #rospy.loginfo('__CidStatusHandler from %s'%(bid))
        self.__publishBeaconStatusArray()
        self.__publishBeaconPoseArray()

    '''
    ========================================
    DAT Protocol Messages
    ========================================
    '''
    # Name          : __CidRcvDatHandler
    # Description   :
    # return        : void
    def __CidRcvDatHandler(self, in_raw_message):
        cidDataRcv = CID_DAT_RECEIVE()  #  Create message handler objects to decode the received message
        cidDataRcv.decode_response(in_raw_message)
        bid = BID_E(cidDataRcv.resp_msg_acofix_t.dest_id_e.Bid).encode_data_dec()
        self.bcn_rxbuff_dict[bid]['data'] = cidDataRcv.resp_msg_packet_data # Store the received data into a local buffer
        rospy.loginfo('__CidRcvDatHandler %s'%(self.bcn_rxbuff_dict[bid]['data']))
        # Verify if the data has to be published. This is only required on the slave side of the communication.
        if((self.bcn_type == 'X110') and (cidDataRcv.resp_msg_packet_len != 0)):
            remote_gps = bcn_remote_gps()   #  Create an object to publish the data through a topic
            id,data = AppMsg_GpsHead().decode_message(self.bcn_rxbuff_dict[bid]['data'])
            remote_gps.long     = data[0]
            remote_gps.lat      = data[1]
            remote_gps.heading  = data[2]
            self.bcnRemoteGpsPub.publish(remote_gps)
        #self.__publishRxBuffer()

    def __CidRcvDatError(self, in_raw_message):
        rospy.loginfo('__CidRcvDatError')

    '''
    ========================================
    Ping Protocol Messages
    ========================================
    '''
    # Name          : __CidPingErrorHandler
    # Description   : Triggered whenever an error while using the ping protocol
    #                 is detected.
    # return        : void
    def __CidPingErrorHandler(self, in_raw_message):
        pingErrHdlr = CID_PING_ERROR()
        pingErrHdlr.decode_response(in_raw_message)
        rospy.loginfo('__CidPingErrorHandler %s'%(pingErrHdlr.resp_msg_cst_e.Cst))
        self.enable_dispatcher = True

    # Name          : __CidPingSendHandler
    # Description   : Acknowledgment of PING_SEND command.
    # return        : void
    def __CidPingSendHandler(self, in_raw_message):
        pingSendHdlr = CID_PING_SEND()
        pingSendHdlr.decode_response(in_raw_message)
        rospy.loginfo('__CidPingSendHandler %s'%(pingSendHdlr.resp_msg_cst_e.Cst))

    # Name          : __CidPingRespHandler
    # Description   :
    # return        : void
    def __CidPingRespHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        pingRespHdlr =  CID_PING_RESP()
        bidHdlr = BID_E()
        # Decode the incomming message
        pingRespHdlr.decode_response(in_raw_message)
        # 07/28 JFCG
        # Get the index of the remote index of the beacon sending the information
        bid = BID_E(pingRespHdlr.resp_msg_acofix_t.src_id_e.Bid).encode_data_dec()
        # Remote beacon data
        self.bcn_pose_dict[bid]['yaw']   = 361
        self.bcn_pose_dict[bid]['pitch'] = 361
        self.bcn_pose_dict[bid]['roll']  = 361
        self.bcn_pose_dict[bid]['supp_volt'] = 14
        self.bcn_pose_dict[bid]['x']     = pingRespHdlr.resp_msg_acofix_t.position_easting_i16/10.0
        self.bcn_pose_dict[bid]['y']     = pingRespHdlr.resp_msg_acofix_t.position_northing_i16/10.0
        rospy.loginfo(pingRespHdlr.resp_msg_acofix_t.position_depth_i16)
        self.bcn_pose_dict[bid]['z']     = pingRespHdlr.resp_msg_acofix_t.position_depth_i16/10.0
        self.bcn_pose_dict[bid]['azim']  = pingRespHdlr.resp_msg_acofix_t.usbl_azimuth_i16/10.0
        self.bcn_pose_dict[bid]['elev']  = pingRespHdlr.resp_msg_acofix_t.usbl_elevation_i16/10.0
        self.bcn_pose_dict[bid]['range'] = pingRespHdlr.resp_msg_acofix_t.range_dist_u16/10.0
        self.__publishBeaconPoseArray()
        #rospy.loginfo('__logPoseData from __CidPingRespHandler')
        self.__logPoseData(0)
        self.enable_dispatcher = True
        #rospy.loginfo('__CidPingRespHandler')
    '''
    ========================================
    Acoustic Transceiver Messages
    ========================================
    '''
    # Name          : __CidXcvrRxRespHandler
    # Description   :
    # return        : void
    def __CidXcvrRxRespHandler(self,in_raw_message):
        # Create handlers for the objects to manage the messages received
        xcvrRxRespHdlr =  CID_XCVR_RX_RESP()
        # Decode the incomming message
        xcvrRxRespHdlr.decode_response(in_raw_message)
        bid = BID_E(xcvrRxRespHdlr.resp_msg_acofix_t.src_id_e.Bid).encode_data_dec()
        #self.bcn_pose_dict[bid]['yaw']   = 361
        #self.bcn_pose_dict[bid]['pitch'] = 361
        #self.bcn_pose_dict[bid]['roll']  = 361
        #self.bcn_pose_dict[bid]['supp_volt'] = 14
        #self.bcn_pose_dict[bid]['x'] = xcvrRxRespHdlr.resp_msg_acofix_t.position_easting_i16/10
        #self.bcn_pose_dict[bid]['y'] = xcvrRxRespHdlr.resp_msg_acofix_t.position_northing_i16/10
        #self.bcn_pose_dict[bid]['z'] = xcvrRxRespHdlr.resp_msg_acofix_t.position_depth_i16/10
        #self.__publishBeaconPoseArray()
        #self.__logPoseData(0)
        # It is not necessary to log data at this point, __CidNavQryRespHandler
        # will be called after and data will be logged there
        rospy.loginfo('__CidXcvrRxRespHandler from %d'%(bid))

    def __CidXcvrRxReqHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        xcvrRxReqHdlr =  CID_XCVR_RX_REQ()
        # Decode the incomming message
        xcvrRxReqHdlr.decode_response(in_raw_message)
        msgenc = xcvrRxReqHdlr.resp_msg_aco_msg_t.msg_pload_au8
        msg = ''
        for num in msgenc:
            msg += chr(num)
        bid = xcvrRxReqHdlr.resp_msg_aco_msg_t.src_id_e.Bid
        #rospy.loginfo('__CidXcvrRxReqHandler: Message received %s'%(msg))
        #rospy.loginfo('__CidXcvrRxReqHandler: From bid %s'%(msg))

    # Name          : __CidXcvrTxMsgHandler
    # Description   :
    # return        : void
    def __CidXcvrTxMsgHandler(self, in_raw_message):
        # Create  handler for the received message
        xcvrTxMsgHdlr = CID_XCVR_TX_MSG()
        bid = BID_E(xcvrTxMsgHdlr.resp_msg_aco_msg_t.src_id_e.Bid).encode_data_dec()
        msg = xcvrTxMsgHdlr.resp_msg_aco_msg_t.msg_pload_au8
        #rospy.loginfo('__CidXcvrTxMsgHandler : %s'%(msg))
        #rospy.loginfo('__CidXcvrTxMsgHandler : %s'%(bid))

    # Name          : __CidXcvrFixHandler
    # Description   :
    # return        : void
    def __CidXcvrFixHandler(self, in_raw_message):
        pass
        #rospy.loginfo('__CidXcvrFixHandler')

    # Name          : __CidXcvrUsblHandler
    # Description   :
    # return        : void
    def __CidXcvrUsblHandler(self, in_raw_message):
        # Create  handler for the received message
        cidXcvrUsbl = CID_XCVR_USBL()
        # Decode the incomming message
        cidXcvrUsbl.decode_response(in_raw_message)
        # reassign decoded data to local variables
        sig_peak = cidXcvrUsbl.resp_msg_xcor_sig_peak_f
        thresh = cidXcvrUsbl.resp_msg_xcor_threshold_f
        cross_point = cidXcvrUsbl.resp_msg_xcor_cross_point_u16
        cross_mag = cidXcvrUsbl.resp_msg_xcor_cross_mag_f
        detect = cidXcvrUsbl.resp_msg_xcor_detect_u16
        length = cidXcvrUsbl.resp_msg_xcor_length_u16
        xcor_data_af = ','.join([str(x) for x in cidXcvrUsbl.resp_msg_xcor_data_af])
        channels = cidXcvrUsbl.resp_msg_channels_u8
        rssi = ','.join([str(x) for x in cidXcvrUsbl.resp_msg_channel_rssi_au16])
        bslns = cidXcvrUsbl.resp_msg_baselines_u8
        phase_ang = ','.join([str(x) for x in cidXcvrUsbl.resp_msg_phase_angle_af])
        azim = cidXcvrUsbl.resp_msg_signal_azimuth_i16/10.0
        elev = cidXcvrUsbl.resp_msg_signal_elevation_i16/10.0
        fit_error = cidXcvrUsbl.resp_msg_signal_fit_error_f
        self.fit_err = fit_error
        # pass local variables to logger
        if(self.usbl_logger != 0):
            self.usbl_logger.logData((sig_peak,thresh,cross_point,cross_mag,detect,length,xcor_data_af,channels,rssi,bslns,phase_ang,azim,elev,fit_error))
        rospy.loginfo('__CidXcvrUsblHandler - fit_error: %f'%(fit_error))
    '''
    ========================================
    NAV Protocol Messages
    ========================================
    '''
    def __CidNavQrySend(self, in_raw_message):
        pass
        #self.enable_dispatcher = True
        #rospy.loginfo('__CidNavQrySend')

    # Name          : __CidNavQryRespHandler
    # Description   :
    # return        : void
    def __CidNavQryRespHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        cidNavQryResp =  CID_NAV_QUERY_RESP()
        # Decode the incomming message
        cidNavQryResp.decode_response(in_raw_message)
        # Get the index of the remote index of the beacon sending the information
        bid = BID_E(cidNavQryResp.resp_msg_acofix_t.src_id_e.Bid).encode_data_dec()
        # Remote beacon data
        self.bcn_pose_dict[bid]['yaw']        = cidNavQryResp.resp_msg_remote_yaw_i16/10
        self.bcn_pose_dict[bid]['pitch']      = cidNavQryResp.resp_msg_remote_pitch_i16/10
        self.bcn_pose_dict[bid]['roll']       = cidNavQryResp.resp_msg_remote_roll_i16/10
        self.bcn_pose_dict[bid]['x']          = cidNavQryResp.resp_msg_acofix_t.position_easting_i16/10
        self.bcn_pose_dict[bid]['y']          = cidNavQryResp.resp_msg_acofix_t.position_northing_i16/10
        self.bcn_pose_dict[bid]['z']          = cidNavQryResp.resp_msg_remote_depth_i32/10.0
        self.bcn_pose_dict[bid]['azim']       = cidNavQryResp.resp_msg_acofix_t.usbl_azimuth_i16/10.0
        self.bcn_pose_dict[bid]['elev']       = cidNavQryResp.resp_msg_acofix_t.usbl_elevation_i16/10.0
        self.bcn_pose_dict[bid]['range']      = cidNavQryResp.resp_msg_acofix_t.range_dist_u16/10.0
        self.bcn_pose_dict[bid]['supp_volt']  = cidNavQryResp.resp_msg_remote_supply_u16/1000
        self.bcn_pose_dict[bid]['temp']       = cidNavQryResp.resp_msg_remote_temp_i16/10
        self.bcn_rxbuff_dict[bid]['data']     = cidNavQryResp.resp_msg_packet_data_au8
        # Publish the data to ROS, so the rest of the node can read the received
        # position of the beacons
        self.__publishBeaconStatusArray()
        self.__publishBeaconPoseArray()
        self.__publishRxBuffer()
        # Log data into csv file
        rospy.loginfo('__logPoseData from __CidNavQryRespHandler: %s'%(self.bcn_rxbuff_dict[bid]['data']))
        self.__logPoseData(0) # Log for all the beacons
        # Reenable tranmission of dispatcher since a final response has been
        # received
        self.enable_dispatcher = True

    # Name          : __CidNavErrorHandler
    # Description   :
    # return        : void
    def __CidNavErrorHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        cidNavError =  CID_NAV_ERROR()
        # Decode the incomming message
        cidNavError.decode_response(in_raw_message)
        #rospy.loginfo('__CidNavErrorHandler')
        # Reenable tranmission of dispatcher since a final response has been
        # received
        self.enable_dispatcher = True

    # Name          : __CidNavQryReqHandler
    # Description   :
    # return        : void
    def __CidNavQryReqHandler(self, in_raw_message):
        # Create handlers for the objects to manage the messages received
        cidNavqRryReqHdlr = CID_NAV_QUERY_REQ()
        # Decode the incomming message
        cidNavqRryReqHdlr.decode_response(in_raw_message)
        bid = BID_E(cidNavqRryReqHdlr.resp_msg_acofix_t.src_id_e.Bid).encode_data_dec()
        self.bcn_rxbuff_dict[bid]['data'] = cidNavqRryReqHdlr.resp_msg_packet_data_au8
        # Output decoded data
        self.__publishRxBuffer()
        #rospy.loginfo('__CidNavQryReqHandler [%d] :%s'%(cidNavqRryReqHdlr.resp_msg_packet_len_u8,cidNavqRryReqHdlr.resp_msg_packet_data_au8))

    '''
    ================================================================================
    Main Node Task
    ================================================================================
    '''

    def main_task(self):
        # Declare the communicaton handler object in order to start comm port.
        # after object creation it is possible to reassign callbacks that are
        # required to be handled here.
        self.msg_hdlr = MsgHandler()

        # Reassign the communication handlers
        self.msg_hdlr.CmdHdlrDict['CID_SETTINGS_GET'] = self.__CidSettingsGetHandler

        # Define the activities that will run in the node. Create a dictionary
        # that contains the function to be run and the time to wait until the
        # next function in the list runs.
        elapsed_time = 0
        trigger_fun = True
        schdl_idx = 0
        if(self.bcn_type == 'X150'):
            if (self.wrk_mode == 'POS_TRACK_PING'):
                schedule_tbl = {0:{'fun':self.sendLocalPosToAllBeacons, 'post_time':3600}}
            elif (self.wrk_mode == 'POS_TRACK_PING_'):
                schedule_tbl = {0: {'fun':self.pingTgt1,        'post_time':3600},
                                1: {'fun':self.pingTgt2,        'post_time':3600},
                                2: {'fun':self.pingRoughie,     'post_time':3600},
                                3: {'fun':self.pingRoughie,     'post_time':3600},
                                4: {'fun':self.pingRoughie,     'post_time':3600},
                                5: {'fun':self.pingRoughie,     'post_time':3600},
                                6: {'fun':self.pingRoughie,     'post_time':3600},
                                7: {'fun':self.pingRoughie,     'post_time':3600},
                                8: {'fun':self.pingRoughie,     'post_time':3600},
                                9: {'fun':self.pingRoughie,     'post_time':3600},
                                10:{'fun':self.pingRoughie,     'post_time':3600},
                                11:{'fun':self.pingRoughie,     'post_time':3600}}
            elif(self.wrk_mode == 'POS_TRACK_COMM_NAV'):
                schedule_tbl = {0:{'fun':self.dispatchMessages,  'post_time':3600}}
            else:
                rospy.loginfo('Not a valid option for beacon X150')
        else:
            if (self.wrk_mode == 'SLAVE'):
                schedule_tbl = {0:{'fun':self.dispatchMessages,   'post_time':1000}}
            else:
                rospy.loginfo('Not a valid option for beacon X110')

        # Once we are ready to handle communication we can proceed to open
        # the serial port.
        rospy.loginfo('Connecting to %s'%(self.bcn_com_port[self.comm_pt_no]))
        self.comm_hdlr = CommHandler(comm_port = self.bcn_com_port[self.comm_pt_no], msg_handler = self.msg_hdlr)
        if(self.comm_hdlr.ser_port == 0):
            self.__TerminateNode('No COM Port available, node will be terminated.')
        else:
            rospy.loginfo('Connection has been stablished successfully.')
            self.comm_hdlr.startCommHandler()
        # Start with a reboot to prevent any possible error
        rospy.loginfo('Reboot the system')
        self.rebootBeacon()
        # Start ROS while loop
        rospy.loginfo('Getting local configuration')
        # lock system execution until local beacon information is received.
        retry_ctr = 10
        glbl_ctr = 0
        while(self.local_bid == ''):
            retry_ctr = retry_ctr - 1
            if(retry_ctr == 0):
                self.getLocalConfiguration()
                retry_ctr = 20
                glbl_ctr = glbl_ctr + 1
                if( glbl_ctr > 20):
                    self.__TerminateNode("Beacon not detected, node will be terminated.")
            self.rate.sleep()
        self.connected_flag = True
        rospy.loginfo('Connected to Beacon: %s'%(self.local_bid))

        # register the rest of the handlers once we know what beacon we are connected to
        self.msg_hdlr.CmdHdlrDict['CID_STATUS'] = self.__CidStatusHandler
        # Dat protocol messages
        self.msg_hdlr.CmdHdlrDict['CID_DAT_RECEIVE'] = self.__CidRcvDatHandler
        self.msg_hdlr.CmdHdlrDict['CID_DAT_ERROR'] = self.__CidRcvDatError
        # Ping protocol messages
        self.msg_hdlr.CmdHdlrDict['CID_PING_SEND'] = self.__CidPingSendHandler
        self.msg_hdlr.CmdHdlrDict['CID_PING_ERROR'] = self.__CidPingErrorHandler
        self.msg_hdlr.CmdHdlrDict['CID_PING_RESP'] = self.__CidPingRespHandler
        # Acoustic Transceiver Messages
        self.msg_hdlr.CmdHdlrDict['CID_XCVR_FIX'] = self.__CidXcvrFixHandler
        self.msg_hdlr.CmdHdlrDict['CID_XCVR_USBL'] = self.__CidXcvrUsblHandler
        self.msg_hdlr.CmdHdlrDict['CID_XCVR_RX_RESP'] = self.__CidXcvrRxRespHandler
        self.msg_hdlr.CmdHdlrDict['CID_XCVR_RX_REQ'] = self.__CidXcvrRxReqHandler
        #self.msg_hdlr.CmdHdlrDict['CID_XCVR_TX_MSG'] = self.__CidXcvrTxMsgHandler
        # Nav protocol messages
        self.msg_hdlr.CmdHdlrDict['CID_NAV_QUERY_SEND'] = self.__CidNavQrySend
        self.msg_hdlr.CmdHdlrDict['CID_NAV_QUERY_RESP'] = self.__CidNavQryRespHandler
        self.msg_hdlr.CmdHdlrDict['CID_NAV_QUERY_REQ'] = self.__CidNavQryReqHandler
        self.msg_hdlr.CmdHdlrDict['CID_NAV_ERROR'] = self.__CidNavErrorHandler

        # Start ROS main loop and set reset timer
        #self.start_time = time.clock()
        self.start_time = time.time()
        while not rospy.is_shutdown():
            # wait for the time defined in the post_time parameter
            #rospy.loginfo_throttle(10, 'While loop running time.clock: %d'%(elapsed_time))
            if trigger_fun == True:
                # run the function associated to the current index value
                schedule_tbl[schdl_idx]['fun']()
                trigger_fun = False
            else:
                if(elapsed_time < schedule_tbl[schdl_idx]['post_time']):
                    pass
                else:
                    # once the wait time is elapsed, reset the timer and move on to the next
                    # activity in the schedule table
                    self.start_time = time.time()
                    elapsed_time = 0
                    trigger_fun = True
                    schdl_idx = schdl_idx + 1
                    # if there is no other entry in the schedule table, start over.
                    if(schdl_idx >= len(schedule_tbl)):
                        schdl_idx = 0
            elapsed_time = int(1000*(time.time()-self.start_time))
            # Put the node to sleep fo the time associated to the execution
            # frequency defined in rospy.Rate(frequency)
            self.rate.sleep()

if __name__ == '__main__':
    Beacon().main_task()
