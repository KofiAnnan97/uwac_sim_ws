#!/usr/bin/env python3

import rospy
import time
import math
import os

from sensor_msgs.msg import FluidPressure, Imu

LOC_TOPIC = ''
DATA_TOPIC = ''
NEIGHBOR_TOPIC = ''
PRESSURE_TOPIC = ''

LAYER_NEIGHBOR_LIMIT = 3

class BeaconNode:
    def __init__(self, layer_count, max_depth):
        self.id = None
        self.neighbors = []
        self.curr_depth = 0

        self.num_of_layers = layer_count
        self.layer_idx = 0
        self.layer_depths = [0]*layer_count
        
        layer_size = max_depth/layer_count
        for i in range(len(layer_count)-1):
            self.layer_depths[i] = 0 if i == 0 else self.layer_depths[i-1] + layer_size

    def start(self):
        # Publishers
        loc_pub = rospy.Publisher(queue_size=1)

        # Subscribers
        rospy.Subscriber(DATA_TOPIC, self.__data_cbk)
        rospy.Subscriber(NEIGHBOR_TOPIC, self.__neighbor_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)

    def __data_cbk(self, data):
        pass

    def __neighbor_cbk(self, data):
        pass

    def __pressure_cbk(self, data):
        self.water_pressure = data
        try:
            standardPressure = 101.325                    # data found in glider_hybrid_whoi/glidereckoining/nodes/deadreckoning_estimator.py 
            KPaPerM = 9.80838
            self.depth_estimate = round((self.water_pressure.fluid_pressure - standardPressure)/KPaPerM, 2)
        except:
            pass

    def send_location(self):
        pass

    def is_layer_too_dense(self):   
        count = 0     
        for neighbor in self.neighbors:
            if math.fabs(neighbor.depth - self.depth) > 10:
                count += 1
        if count > LAYER_NEIGHBOR_LIMIT:
            self.move_layer()

    def move_layer(self):
        pass

    def send_to_csv(self, name):
        import csv
        

    def run(self):
        if not rospy.is_shutdown:
            pass

if __name__ == "__main__":
    bn = BeaconNode(3, 10)
    bn.start()
    bn.run()