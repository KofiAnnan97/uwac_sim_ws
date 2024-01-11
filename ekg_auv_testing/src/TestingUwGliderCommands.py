#!/usr/bin/env python3

import rospy
import math
import time
from frl_vehicle_msgs.msg import UwGliderCommand, UwGliderStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from tf.transformations import euler_from_quaternion
from graphing import Graphing3D

AUV_TRUE_POSE_1 = "true_pose_1"
AUV_EST_POSE_1 = "estimate_pose_1"

class UWGCTester():
    def __init__(self):
        self.cmd = UwGliderCommand()
        self.grapher = Graphing3D()
        self.grapher.add_path(AUV_TRUE_POSE_1, "True Pose AUV 1")
        # self.grapher.add_path(AUV_EST_POSE_1, "Estimated Pose AUV 1")

        #Sensor Data
        self.water_pressure = FluidPressure() # FluidPressure msg uses KiloPascals
        self.imu = Imu()
        self.gps = NavSatFix()

        #Ground Truth
        self.true_pose = PoseStamped()
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])
        self.uwg_status = UwGliderStatus()

        #Estimates
        self.depth_estimate = float("inf")
        self.pose_estimate = None

        # Values
        self.time0 = rospy.Time.now()
        self.yaw_in_deg = 90


    def init_app(self):
        from sys import argv
        UWGC_TOPIC = f'/{argv[1]}/kinematics/UwGliderCommand'
        UWGS_TOPIC = f'/{argv[1]}/kinematics/UwGliderStatus'
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose'
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'

        #Initialize Publishers
        self.command_pub = rospy.Publisher(UWGC_TOPIC, UwGliderCommand, queue_size=1)
        #Initializee Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(UWGS_TOPIC, UwGliderStatus, self.__status_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)
        

    def __true_pose_cbk(self, data):
        self.true_pose = data
        t_pos = self.true_pose.pose.position 
        self.grapher.add_path_point(AUV_TRUE_POSE_1, t_pos.x, t_pos.y, t_pos.z)
        #rospy.loginfo(f"True Pose: ({self.true_pose.pose.position.x},{self.true_pose.pose.position.y},{self.true_pose.pose.position.z})")
    
    def __status_cbk(self, data):
        self.uwg_status = data
        # rospy.loginfo(self.status_msg(self.uwg_status))
    
    def __pressure_cbk(self, data):
        self.water_pressure = data
        try:
            standardPressure = 101.325                    # data found in glider_hybrid_whoi/glidereckoining/nodes/deadreckoning_estimator.py 
            KPaPerM = 9.80838
            self.depth_estimate = round((self.water_pressure.fluid_pressure - standardPressure)/KPaPerM, 2)
            #rospy.loginfo(f"Depth Estimate: {self.depth_estimate}, Actual Depth: {self.true_pose.pose.position.z}")
        except:
            pass

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

    def update_pose_estimate(self, aoa_rad):
        orient = self.imu.orientation
        row, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        if pitch < 0:
            glide_angle = pitch - aoa_rad
        else:
            glide_angle = pitch + aoa_rad

        curr_time = rospy.Time.now()
        dt = curr_time.secs - self.time0.secs
        if dt < 1.0e-3:
            return
        self.time0 = curr_time
        if abs(pitch) < math.pi/100.0:
            v_x = 0.0
            v_y = 0.0
        else:
            deg = yaw * (180/math.pi)
            dz = self.depth_estimate
            v_x = dz/(dt*math.tan(glide_angle))*math.sin(math.pi/2.0-yaw)
            v_y = dz/(dt*math.tan(glide_angle))*math.cos(math.pi/2.0-yaw)
        d_x = v_x * dt
        d_y = v_y * dt

        if self.pose_estimate is None:
            self.pose_estimate = self.init_pose
        else:
            self.pose_estimate.pose.position.x += d_x/math.cos(yaw*2.0*math.pi/360)
            self.pose_estimate.pose.position.y += d_y/math.sin(yaw*2.0*math.pi/360)
            self.pose_estimate.pose.position.z = self.depth_estimate
        
        rospy.loginfo(f"Pose Estimate: ({self.pose_estimate.pose.position.x}, { self.pose_estimate.pose.position.y}, {self.pose_estimate.pose.position.z})")
        return self.pose_estimate if not None else PoseStamped()

    def status_msg(self, status):
        return f"""
        Roll:        {status.roll}
        Pitch:       {status.pitch}
        Heading:     {status.heading}
        Depth:       {status.depth}
        Motor Power: {status.motor_power}
        Pump Volume: {status.pumped_volume}
        """

    def command_msg(self, command):
        return f"""
        Target Pitch Value:   {command.target_pitch_value}
        Target Motor Command: {command.target_motor_cmd}
        Target Heading:       {command.target_heading}
        Rudder Angle:         {command.rudder_angle}
        Target Rudder Angle:  {command.target_rudder_angle}
        Target Pump Volume:   {command.target_pumped_volume}
        """

    def pitch_up(self):
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = -0.2
        self.cmd.target_pumped_volume = 100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = math.pi/2.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 30.0
        msg = self.command_msg(self.cmd)
        #rospy.loginfo(msg)
        self.command_pub.publish(self.cmd)

    def pitch_down(self):
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0.4
        self.cmd.target_pumped_volume = -100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = math.pi/2.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 60.0
        msg = self.command_msg(self.cmd)
        #rospy.loginfo(msg)
        self.command_pub.publish(self.cmd)

    def forward_thrust(self):
        #print("forward thrust")
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0.2
        self.cmd.target_pumped_volume = 50 #100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = math.pi/2.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 30.0
        msg = self.command_msg(self.cmd)
        #rospy.loginfo(msg)
        self.command_pub.publish(self.cmd) 

    def turn_right(self):
        #print("forward thrust")
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0.2
        self.cmd.target_pumped_volume = 50 #100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = math.pi/18.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 20.0
        msg = self.command_msg(self.cmd)
        #rospy.loginfo(msg)
        self.command_pub.publish(self.cmd) 

    def turn_left(self):
        #print("forward thrust")
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0
        self.cmd.target_pumped_volume = 0 #100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = 160 #4.0*math.pi/9.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 20.0
        msg = self.command_msg(self.cmd)
        #rospy.loginfo(msg)
        self.command_pub.publish(self.cmd)

    def stop(self):
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.target_motor_cmd = 0
        self.command_pub.publish(self.cmd)

    def depth_controller(self, target_depth, start_time):
        now = rospy.Time.now()
        if now.secs - start_time.secs >= 10:
            self.stop()
            self.grapher.show_plot("AUV Depth Controller Test")
            rospy.signal_shutdown("Task Complete")

        pitch_val = 0
        pump_val = 0
        if self.depth_estimate != float("inf"):
            offset = 2
            last_pv = 0
            diff = round(0.15*((target_depth + offset) - self.depth_estimate), 2)
            if diff > 0.4:
                pitch_val = 0.4
            elif diff < -0.4:
                pitch_val = -0.4
            #elif diff < 0.1 and diff > -0.1:
            #    pitch_val = 0
            else:
                pitch_val = diff

            if math.fabs(target_depth - self.depth_estimate) < 0.3:
                pump_val = 0
            elif self.depth_estimate < target_depth:
                pump_val = -100
            elif self.depth_estimate > target_depth:
                pump_val = 100

            # if(last_pv != pitch_val):
            #    print(f"Target: {target_depth}, Estimate: {self.depth_estimate}, Pitch: {pitch_val}, Pump: {pump_val}")
            last_pv = pitch_val

        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = pitch_val
        self.cmd.target_pumped_volume = pump_val
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = math.pi/2.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 20.0
        self.command_pub.publish(self.cmd)

    def circle(self):
        self.yaw_in_deg -= 1 
        if self.yaw_in_deg < 0:
            self.yaw_in_deg = 360
        elif self.yaw_in_deg > 360:
            self.yaw_in_deg = 0

        yaw_in_rad = self.yaw_in_deg*2.0*math.pi/360.0
        self.update_pose_estimate(yaw_in_rad)

        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0
        self.cmd.target_pumped_volume = 0 #100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = self.yaw_in_deg #160 #4.0*math.pi/9.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 20.0
        self.command_pub.publish(self.cmd)

    def circle_test(self, start_time):
        curr_time = rospy.Time.now()
        if curr_time.secs - start_time.secs < 60:
            self.circle()
            time.sleep(3)
        else:
            self.stop()
            self.grapher.show_plot("AUV Circle Test")
            rospy.signal_shutdown("Task Complete")

    def helix(self):
        pose_est = self.get_pose_estimate()
        if pose_est is not None:
            e_pos = pose_est.pose.position
            self.grapher.add_path_point(AUV_EST_POSE_1, e_pos.x, e_pos.y, self.depth_estimate)
        print("Helix")

    def test_commands(self):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.circle_test(start_time)
            # self.depth_controller(25, start_time)
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("UwGlider_tester")
        tester = UWGCTester()
        tester.init_app()
        tester.test_commands()
    except rospy.ROSInterruptException:
        rospy.loginfo("UwGliderCommand Tester terminated.")