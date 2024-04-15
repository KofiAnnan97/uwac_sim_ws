#!/usr/bin/env python
# rosrun seatr_pkg CustCommTest.py _beacon_type:=X110  
import numpy as np
import rospy
import time
from seatrac_pkg.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose

from MessageTypes import *
from CommHandler import *
from CsvLogger import *

class Test(object):
    # Name          : __init__
    # Description   : class constructor
    def __init__(self):
        # Node initialization and definition of execution rate parameters of node
        self.ctr = 0
        self.comm_logger = 0
        self.bcnTxPub = rospy.Publisher('beacon_tx_buffer', bcn_frame_array, queue_size=10)
        rospy.Subscriber('beacon_rx_buffer', bcn_frame_array, self.__rxCallback)
        rospy.init_node('TestNode', anonymous=True)
        self.rate = rospy.Rate(0.5)
        self.bcn_type = rospy.get_param('~beacon_type')
        self.comm_pt_no = rospy.get_param('~comm_port')
        
        currentFolder = os.path.dirname(os.path.realpath(__file__))
        if(self.bcn_type == 'X150'):
            self.comm_logger = CsvLogger(path = currentFolder + '/log/x150/' + self.comm_pt_no + '/comm', loggername = 'comm_x150', header='bid,data',format ='%s, %s')
        else:
            self.comm_logger = CsvLogger(path = currentFolder + '/log/x110/' + self.comm_pt_no + '/comm', loggername = 'comm_x110', header='bid,data',format ='%s, %s')
    def runner(self):
        # Start ROS main loop
        while not rospy.is_shutdown():
            frame_array = bcn_frame_array()
            frame = bcn_frame()
            if(self.bcn_type == 'X150'):
                frame.bid = 'BEACON_ID15'
                frame.data = 'Master to slave  - ' + str(self.ctr)
            else:
                frame.bid = 'BEACON_ID1'
                frame.data = 'docking_done'
            self.ctr = self.ctr + 1
            frame_array.frame.append(frame)
            self.bcnTxPub.publish(frame_array)
            self.rate.sleep()

    def __rxCallback(self,msg):
        for frame in msg.frame:
            self.comm_logger.logData((frame.bid,frame.data))
            rospy.loginfo('__rxCallback %s, %s'%(frame.bid,frame.data))

if __name__ == '__main__':
    Test().runner()
