#!/usr/bin/env python3
import rospy
from datetime import datetime, timezone
from math import sin, cos, atan2, sqrt, radians, degrees, asin, exp, tan, atan

import io
import serial
import serial.tools.list_ports as port_list
import pynmea2

from seatrac_pkg.msg import *

# Object that derives local position from GPS Coordinates
class GPSConverter:
    def __init__(self, lat = None, long = None):
        self.ratio = 1 #0.001               # Convert km to m
        self.earth_radius = 6378.137 
        self.earth_semi_major = 6378.137
        self.earth_semi_minor = 6356.752

        self.e_sqrt = 1 - (self.earth_semi_minor**2/self.earth_semi_major**2)
        self.f = 1 - (self.earth_semi_minor/self.earth_semi_major)

        if lat is not None and long is not None:
            self.set_origin(lat, long)
        else:
            self.origin_pt = None

    def set_origin(self, lat, long):
        self.origin_pt = {
            'latitude': lat,
            'longitude': long
        }

    def set_scaling(self, val):
        self.ratio = val

    """
    Formula based on Haversing formula (https://en.wikipedia.org/wiki/Haversine_formula)
    https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters#11172685
    Returns a distance in km
    """
    def get_distance_haversine(self, lat, long):
        if self.origin_pt is not None:
            d_lat = radians(lat - self.origin_pt['latitude'])
            d_long =radians(long - self.origin_pt['longitude'])
            a = sin(d_lat/2) ** 2 + cos(lat) * cos(self.origin_pt['latitude']) * sin(d_long/2) ** 2
            dist = 2 * asin(sqrt(a)) * self.earth_radius
            return dist
        return None
    
    def get_bearing(self, lat, long):
        if self.origin_pt is not None:
            lat1 = radians(self.origin_pt['latitude'])
            long1 = radians(self.origin_pt['longitude'])
            lat2 = radians(lat)
            long2 = radians(long)

            x = cos(lat2) * sin(long2 - long1)
            y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1) 
            bearing = atan2(x, y)
            bearing = degrees(bearing)
            bearing = (bearing + 360 ) % 360
            return bearing
        return None
    
    # NEEDS VERIFICATION
    def get_local_coord(self, lat, long):
        try:
            dist = self.ratio * self.get_distance_haversine(lat, long)
            bearing = self.get_bearing(lat, long)
            print(dist, bearing)
            offset = 90
            x = dist * cos(bearing + offset)
            y = dist * sin(bearing + offset)
            return (x, y)
        except Exception as e:
            print(e)
            return None 

    # FAIRLY ACURRATE 
    def from_gps_to_ecef_simple(self, lat, long):   
        lat = radians(lat)
        long = radians(long)
        x = self.earth_semi_minor * cos(lat) * cos(long)
        y = self.earth_semi_major * cos(lat) * sin(long)
        return (x, y)

    # Based on the conversion from GPS coordinates to Earth-centric Earth-fixed coordinates
    # (https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates)
    def from_gps_to_ecef(self, lat, long, alt):
        #alt = 0
        lat = radians(lat)
        long = radians(long)
        x = (self.__n(lat) + alt) * cos(lat) * cos(long)
        y = (self.__n(lat) + alt) * cos(lat) * sin(long)
        z = ((1-self.f**2)*self.__n(lat) + alt) * sin(lat)
        return (x,y,z)

    def __n(self, coord):
        a = self.earth_semi_major
        n = a / (sqrt(1 - (self.e_sqrt/ (1 + atan(coord)**2))))
        return n

class GPSPoseTracker(object):
    def __init__(self):
        LOC_TOPIC = 'beacon_gps_data'

        self.gps_comm_ports = {
            '0': '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0',
        }
        #self.comm_pt_no = rospy.get_param('~comm_port')
        s_port = self.gps_comm_ports["0"]
        baud = 4800
        # Start serial handler to get GPS data
        self.ser = serial.Serial(port=s_port, baudrate=baud, timeout=1.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))

        # Class for Localization by GPS Coordinates
        self.gl = GPSConverter()
        self.alt = 0

        # Declare publisher
        self.gpsPub = rospy.Publisher(LOC_TOPIC, loc, queue_size=10)
        # Declare subscriber
        rospy.Subscriber(LOC_TOPIC, loc, self.__loc_cbk)
        
        # Node initialization and definition of execution rate parameters of node
        rospy.init_node('GpsTracker', anonymous=True)
        self.rate = rospy.Rate(20)

    def __loc_cbk(self, msg):
        if msg is not None:
            rospy.loginfo('Latitude: %f, Longitude: %f'%(msg.lat,msg.long))
            self.gl.set_origin(msg.lat - 10, msg.long)
            loc_est0 = self.gl.get_local_coord(msg.lat, msg.long)
            rospy.loginfo('Method #0: (%f, %f)'%(loc_est0[0], loc_est0[1]))
            loc_est1 = self.gl.from_gps_to_ecef_simple(msg.lat, msg.long)
            #loc_est1 = self.gl.from_gps_to_ecef_simple(48.8562, 2.3508)
            rospy.loginfo('Method #1: (%f, %f)'%(loc_est1[0], loc_est1[1]))
            loc_est2 = self.gl.from_gps_to_ecef(msg.lat, msg.long, self.alt)
            #loc_est2 = self.gl.from_gps_to_ecef(48.8562, 2.3508, 0.0674)
            rospy.loginfo('Method #2: (%f, %f, %f)'%(loc_est2[0], loc_est2[1], loc_est2[2]))
            print(self.alt)

    def run(self):
        while not rospy.is_shutdown():
            try:
                try:
                    line = self.sio.readline()
                except UnicodeError as e:
                    line = ''
                    rospy.loginfo('{}, bad line?'.format(e))
                msg = pynmea2.parse(line)
                if(type(msg) == pynmea2.types.talker.GGA):
                    #print(msg.fields)
                    #print('%f %s'%(msg.altitude, msg.altitude_units))
                    self.alt = msg.altitude
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

if __name__ == "__main__":
    GPSPoseTracker().run()