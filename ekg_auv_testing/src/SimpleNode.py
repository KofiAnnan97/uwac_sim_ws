#!/usr/bin/env python3

import rospy
import math
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist, PoseWithCovariance, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from std_msgs.msg import String
from rospy_message_converter import message_converter, json_message_converter
from ekg_auv_testing.msg import bcn_frame_array, bcn_frame, bcn_pose_array, bcn_pose, \
                                bcn_remote_gps, bcn_status_array, bcn_status, head, \
                                IverOSI, loc

class SimpleNode:
    def __init__(self) -> None:
        self.true_pose = PoseStamped()
        
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])
        self.curr_pose = self.true_pose #PoseStamped()
        self.imu = Imu()
        self.gps = NavSatFix()
    
    def init_app(self):
        from sys import argv

        # Location Topics
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose' 
        TRUE_EULER_TOPC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/euler'
        ODOM_TOPIC = f'/{argv[1]}/pose_gt'

        # Transciever Topics
        TRANSCEIVER_TOPIC = "/USBL/transceiver_manufacturer_168/command_response"
        INTERROGATION_TOPIC = '/USBL/transceiver_manufacturer_168/interrogation_mode'
        CHANNEL_SWITCH_TOPIC = '/USBL/transceiver_manufacturer_168/channel_switch'

        # Transponder Topics
        TRANSPONDER_TOPIC = "/USBL/transponder_manufacturer_1/command_request"
        COMMON_TOPIC = '/USBL/common_interrogation_ping'
        INDIVIDUAL_TOPIC = '/USBL/transponder_manufacturer_1/individual_interrogation_ping'


        self.rate = rospy.Rate(10)

        # Publishers
        self.odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
        self.channel_pub = rospy.Publisher(CHANNEL_SWITCH_TOPIC, String, queue_size=1)
        self.common_pub = rospy.Publisher(COMMON_TOPIC, String, queue_size=30)

        # Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)

    def __true_pose_cbk(self, data):
        self.true_pose = data

    def __imu_cbk(self, data):
        self.imu = data
        orient = self.imu.orientation
        a_vel = self.imu.angular_velocity
        l_acc = self.imu.linear_acceleration

    def __gps_cbk(self, data):
        try:        
            self.gps = data
        except:
            pass

    def send_pos(self):
        import json
        json_str = json_message_converter.convert_ros_message_to_json(self.curr_pose)
        pos_msg = String()
        pos_msg.data = str(json_str)
        self.common_pub.publish(pos_msg)
        #print(pos_msg.data)

    def run(self):
        rospy.loginfo("Node started.")
        while not rospy.is_shutdown():
            self.send_pos()
            odom = Odometry()
            move = odom.twist.twist
            if(self.true_pose.pose.position.z > -10):
                move.linear.z = -0.5
            elif(self.true_pose.pose.position.z == -10):
                move.linear.z = 0.5
            self.odom_pub.publish(odom)
            self.rate.sleep()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))       

if __name__ == "__main__":
    rospy.init_node("simple_node", anonymous=True)
    node = SimpleNode()
    node.init_app()
    node.run()
    try:
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Glider navigation has been terminated.")