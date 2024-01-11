#!/usr/bin/env python3

import rospy
import math
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist, PoseWithCovariance, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix

class SimpleNode:
    def __init__(self) -> None:
        self.true_pose = PoseStamped()
        
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])
        self.curr_pose = PoseStamped()
        self.imu = Imu()
        self.gps = NavSatFix()
    
    def init_app(self):
        from sys import argv
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose' 
        TRUE_EULER_TOPC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/euler'
        ODOM_TOPIC = f'/{argv[1]}/pose_gt'

        self.rate = rospy.Rate(10)

        # Publishers
        self.odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
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

    def run(self):
        rospy.loginfo("Node started.")
        while not rospy.is_shutdown():
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