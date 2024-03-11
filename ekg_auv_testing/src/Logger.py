import rospy
import rospkg
import os
import csv

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from ekg_auv_testing.msg import VehicleLog

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

    def get_log_path(self):        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("ekg_auv_testing")
        logs_path = os.path.join(pkg_path, "logs")
        return logs_path

    def start(self):
        logs_path = self.get_log_path()
        glider_path = os.path.join(logs_path, self.glider_name)
        if not os.path.exists(glider_path):
            os.makedirs(glider_path)
        file_name = f'{self.timestamp}_{self.glider_name}.csv'
        file_path = os.path.join(glider_path, file_name)
        self.file_path = file_path

        with open(file_path, 'w', newline='') as cw:
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

class BeaconLogger:
    def __init__(self):
        pass