import rospy
import rospkg
import os
import csv

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from uwac_sim.msg import VehicleLog, VehiclePose

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path("uwac_sim")
LOG_PATH = os.path.join(PKG_PATH, "log")

if not os.path.exists(LOG_PATH):
    os.makedirs(LOG_PATH)

class GliderLogger:
    def __init__(self, glider_name, stamp, glider_topic):
        # Variables
        self.is_running = False
        self.glider_name = glider_name
        self.timestamp = stamp
        self.file_path = None
        self.curr_idx = 0

        #print(glider_topic)
        # Subscribers
        rospy.Subscriber(glider_topic, VehicleLog, self.__log_cbk)

    def __log_cbk(self, msg):
        try:
            if self.is_running == True:
                pos = msg.position
                orient = msg.orientation
                ang_vel = msg.angular_velocity
                lin_acc = msg.linear_acceleration
                (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
                with open(self.file_path,'a', newline='') as cw:
                    writer = csv.writer(cw)
                    writer.writerow([msg.header.stamp.secs, round(pos.x, 2),  round(pos.y, 2), round(pos.z, 2), lin_acc.x, lin_acc.y, lin_acc.z, ang_vel.x, ang_vel.y, ang_vel.z, round(roll, 2), round(pitch, 2), round(yaw, 2)])
                    self.curr_idx += 1
        except Exception as e:
            print(e)

    def __imu_cbk(self, msg):
        pass

    #def get_log_path(self):        
    #    rospack = rospkg.RosPack()
    #    pkg_path = rospack.get_path("uwac_sim")
    #    logs_path = os.path.join(pkg_path, "log")
    #    return logs_path
    
    def set_glider_log(self, glider_name, stamp):
        #logs_path = self.get_log_path()
        glider_path = os.path.join(LOG_PATH, glider_name)
        if not os.path.exists(glider_path):
            os.makedirs(glider_path)
        file_name = f'{stamp}_{glider_name}.csv'
        file_path = os.path.join(glider_path, file_name)
        self.file_path = file_path


    def start(self):
        """logs_path = self.get_log_path()
        glider_path = os.path.join(logs_path, self.glider_name)
        if not os.path.exists(glider_path):
            os.makedirs(glider_path)
        file_name = f'{self.timestamp}_{self.glider_name}.csv'
        file_path = os.path.join(glider_path, file_name)
        self.file_path = file_path"""
        self.set_glider_log(self.glider_name, self.timestamp)

        with open(self.file_path, 'w', newline='') as cw:
            writer = csv.writer(cw)
            writer.writerow(["timestamp", "x", "y", "z", "lin_acc_x", "lin_acc_y", "lin_acc_z", "ang_vel_x", "ang_vel_y", "ang_vel_z", "roll", "pitch", "yaw"])

        self.is_running = True

    def retrieve_data_by_time(self, b_stamp, e_stamp):
        pose_log = []
        with open(self.file_path, 'r') as cr:
            reader = csv.reader(cr)
            for row in reversed(list(reader)):
                if row[0] == "timestamp":
                    return pose_log
                elif int(row[0]) <= b_stamp:
                    return pose_log
                elif int(row[0]) > b_stamp and int(row[0]) <= e_stamp: 
                    pose_log.insert(0, row)

class StratsLogger:
    def __init__(self, glider_name, stamp):
        self.is_running = False
        self.glider_name = glider_name
        self.timestamp = stamp
        self.strategies = ['avg', "weighted_avg", "closest_neighbor", "weighted_avg_with_dead_reckoning"]
        self.file_paths = dict()
        for strat in self.strategies:
            self.file_paths[strat] = os.path.join(LOG_PATH, strat)

        # Localization Strategies
        LOC_STRATS_TOPIC = f"{self.glider_name}/pose_estimates"

        rospy.Subscriber(LOC_STRATS_TOPIC, VehiclePose, self.__log_cbk)

    def __log_cbk(self, msg):
        try:
            if self.is_running == True and msg.vehicle_name in self.strategies:
                with open(self.file_paths[msg.vehicle_name],'a', newline='') as cw:
                    writer = csv.writer(cw)
                    writer.writerow([msg.stamp.secs, round(msg.x, 2),  round(msg.y, 2), round(msg.z, 2)])
        except Exception as e:
            print(e)

    def start(self):
        #self.set_glider_log(self.glider_name, self.timestamp)
        for strat_name, log_path in self.file_paths.items():
            if not os.path.exists(log_path):
                os.makedirs(log_path)
            filename = f'{self.timestamp}_{strat_name}.csv'
            file_path = os.path.join(log_path, filename)
            
            with open(file_path, 'w', newline='') as cw:
                writer = csv.writer(cw)
                writer.writerow(["timestamp", "x", "y", "z"])
            log_path = file_path

        self.is_running = True

class BeaconLogger:
    def __init__(self):
        pass