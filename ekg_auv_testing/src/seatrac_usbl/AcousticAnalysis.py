#!/usr/bin/env python
import os
import numpy as np
import rospy
import time
from seatrac_pkg.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, bcn_status, bcn_status_array, loc
#from aut_cat.msg import loc

from MessageTypes import *
from CommHandler import *
from CsvLogger import *
from datetime import datetime
import time

# Added test comment
class AcousticAnalysis(object):
    def __init__(self, analysis_duration = 120.0):
        self.bcn_com_port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        # Variable initialization
        self.analysis_duration = analysis_duration
        self.start_time = 0
        self.comm_hdlr = 0
        self.xcvr_analyse_logger = 0
        self.block_xcvr_analysis_flag = False
        self.gps_pos = {'lat':0.0,'long':0.0}
        # Logger definition
        currentFolder = os.path.dirname(os.path.realpath(__file__))
        self.xcvr_analyse_logger  = CsvLogger(path = currentFolder + '/log/xcvranalyse', loggername = 'xcvranalyse', header='adc_mean,adc_pkpk,adc_rms,rxlvl_pkpk,rxlvl_rms,lat,long',format ='%f, %f, %f, %f, %f, %f, %f')
        # Subscribe to topics
        rospy.Subscriber('beacon_gps_data',loc, self.__gpsCallback)
        rospy.Subscriber('gps',loc, self.__gpsCallback)
        # Xcvr Analysis
        rospy.init_node('XcvrAnalysis', anonymous=True)
        self.rate = rospy.Rate(10)

    def xcvrAnalyse(self):
        if(self.block_xcvr_analysis_flag == False):
            rospy.loginfo('Request')
            xcvrAnalyseHdrl = CID_XCVR_ANALYSE()
            txframe = xcvrAnalyseHdrl.encode_frame()
            self.comm_hdlr.sendMessage(txframe)
            self.block_xcvr_analysis_flag = True

    # Name          : __gpsCallback
    # Description   : Function handler triggered upon the reception of gps data.
    # return        : void
    def __gpsCallback(self,data):
        self.gps_pos['lat'] = data.lat
        self.gps_pos['long'] = data.long

    def __CidXcvrAnalyseHandler(self,in_raw_message):
        xcvrAnalyseHdrl = CID_XCVR_ANALYSE()
        # Decode the incomming message
        rospy.loginfo('Response')
        xcvrAnalyseHdrl.decode_response(in_raw_message)
        adc_mean = xcvrAnalyseHdrl.resp_msg_adc_mean_i16
        adc_pkpk = xcvrAnalyseHdrl.resp_msg_adc_pkpk_i16
        adc_rms = xcvrAnalyseHdrl.resp_msg_adc_rms_u32
        rxlvl_pkpk = xcvrAnalyseHdrl.resp_msg_rx_lvl_pkpk_i16/10
        rxlvl_rms = xcvrAnalyseHdrl.resp_msg_rx_lvl_rms_i16/10
        lat = self.gps_pos['lat']
        long = self.gps_pos['long']
        # log data until the log hadndler has been initialized
        if(self.xcvr_analyse_logger != 0):
            self.xcvr_analyse_logger.logData((adc_mean, adc_pkpk, adc_rms, rxlvl_pkpk, rxlvl_rms, lat, long))
            self.block_xcvr_analysis_flag = False

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
        self.msg_hdlr.CmdHdlrDict['CID_XCVR_ANALYSE'] = self.__CidXcvrAnalyseHandler

        # Define the activities that will run in the node. Create a dictionary
        # that contains the function to be run and the time to wait until the
        # next function in the list runs.
        elapsed_time = 0
        schdl_idx = 0
        schedule_tbl = {0:{'fun':self.xcvrAnalyse, 'post_time':1000}}

        # Once we are ready to handle communication we can proceed to open
        # the serial port.
        self.comm_hdlr = CommHandler(comm_port = self.bcn_com_port, msg_handler = self.msg_hdlr)
        self.comm_hdlr.startCommHandler()

        # Start infinit loop
        self.start_time = time.time()
        while not rospy.is_shutdown():
            # wait for the time defined in the post_time parameter
            elapsed_time = int(1000*(time.time()-self.start_time))
            #rospy.loginfo_throttle(10, 'While loop running time.clock: %d'%(elapsed_time))
            if(elapsed_time < schedule_tbl[schdl_idx]['post_time']):
                # get milliseconds elapsed from previous idx count
                pass
            else:
                # run the function associated to the current index value
                schedule_tbl[schdl_idx]['fun']()
                # once the wait time is elapsed, reset the timer and move on to the next
                # activity in the schedule table
                self.start_time = time.time()
                elapsed_time = 0
                schdl_idx = schdl_idx + 1
                # if there is no other entry in the schedule table, start over.
                if(schdl_idx >= len(schedule_tbl)):
                    schdl_idx = 0
            # Put the node to sleep fo the time associated to the execution
            # frequency defined in rospy.Rate(frequency)
            self.rate.sleep()

if __name__ == '__main__':
    AcousticAnalysis().main_task()
