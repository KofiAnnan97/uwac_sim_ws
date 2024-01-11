#!/usr/bin/env python
import os
import threading
import serial
import serial.tools.list_ports as port_list
from datetime import datetime
import time
from seatrac_pkg.msg import loc
import io
import pynmea2
import rospy
from CsvLogger import *

import logging
import logging.handlers

# GPS  - /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
# x150 - /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
# x110 - /dev/serial/by-id/usb-FTDI_US232R_FT2L0U9L-if00-port0

class GpsTracker(object):
    def __init__(self):
        # Start serial handler to get GPS data
        self.ser = serial.Serial('/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0', 4800, timeout=1.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        # Declare publisher
        self.gpsPub = rospy.Publisher('beacon_gps_data', loc, queue_size=10)
        # Node initialization and definition of execution rate parameters of node
        rospy.init_node('GpsTracker', anonymous=True)
        self.rate = rospy.Rate(20)
        # Create the folder to store logged data
        currentFolder = os.path.dirname(os.path.realpath(__file__))
        self.gps_logger = CsvLogger(path = currentFolder + '/log/gps', loggername = 'local', header='lat,long,gps_qual,num_sats',format ='%f, %f, %s, %s')

    def main_task(self):
        while not rospy.is_shutdown():
            try:
                try:
                    line = self.sio.readline()
                except UnicodeError as e:
                    line = ''
                    rospy.loginfo('{}, bad line?'.format(e))
                msg = pynmea2.parse(line)
                if(type(msg) == pynmea2.types.talker.GGA):
                    self.gps_logger.logData((msg.latitude,msg.longitude, msg.gps_qual, msg.num_sats))
                    #rospy.loginfo('%f, %f, %s, %s'%(msg.latitude,msg.longitude, msg.gps_qual, msg.num_sats))
                    loc_hdlr = loc()
                    loc_hdlr.lat = msg.latitude
                    loc_hdlr.long = msg.longitude
                    self.gpsPub.publish(loc_hdlr)
            except serial.SerialException as e:
                rospy.loginfo('Device error: {}'.format(e))
                break
            except pynmea2.ParseError as e:
                rospy.loginfo('Parse error: {}'.format(e))
                continue
            self.rate.sleep()

if __name__ == '__main__':
    GpsTracker().main_task()
