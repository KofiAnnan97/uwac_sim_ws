#!/usr/bin/env python3

import rospy
import math
import time
import json
from datetime import datetime

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from frl_vehicle_msgs.msg import UwGliderCommand, UwGliderStatus
from rospy_message_converter import message_converter, json_message_converter

from Graphing3D import Graphing3D
from TestWaypointPaths import get_circle_path, get_helix_path, get_double_helix_path
from USBL import Transponder
from Logger import GliderLogger
from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim, VehicleLog


AUV_TRUE_POSE_1 = "true_pose"
AUV_EST_POSE_1 = "dead_reckoning"
AUV_BEACON_POSE_1 = "avg"
AUV_BEACON_POSE_2 = "weighted_avg"
AUV_BEACON_POSE_3 = "closest_neighbor"
AUV_FUSION_POSE_1 = "avg_with_dead_reckoning"
AUV_FUSION_POSE_2 = "weighted_avg_with_dead_reckoning"
AUV_FUSION_POSE_3 = "closest_neighbor_with_dead_reckoning"
#labels = [AUV_TRUE_POSE_1, AUV_EST_POSE_1, AUV_BEACON_POSE_1, AUV_BEACON_POSE_2, AUV_BEACON_POSE_3]
labels = [AUV_TRUE_POSE_1, AUV_EST_POSE_1, AUV_BEACON_POSE_2, AUV_FUSION_POSE_2]


YAW_UPPER_LIMIT = 2.62    # [radians]
YAW_LOWER_LIMIT = 0.52    # [radians]

PITCH_UPPER_LIMIT = 2.62  # [radians]
PITCH_LOWER_LIMIT = 0.52  # [radians]

WAYPOINTS = get_circle_path()
COMPLETION_THRESHOLD = 1.5

INDIVIDUAL_MSG = {"data": 'individual'}
COMMON_MSG = {"data": 'common'}
HELLO_MSG = {"data": 'Send position'}

class WaypointFollower():
    def __init__(self):
        from sys import argv
        self.rate = rospy.Rate(10)

        self.transponder_id = argv[5]
        #self.transponder_model = ""
        self.transceiver_id = "141"
        #self.transceiver_model = ""
        self.rx = Transponder(self.transponder_id, self.transceiver_id, argv[1])

        #Ground Truth
        self.true_pose = PoseStamped()
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])
        self.uwg_status = UwGliderStatus()

        self.aoa_deg = -0.7
        self.prev_depth = float("inf")

        # Deadreckoning Pose Estimation
        self.est_pose = PoseStamped()
        #self.est_pose.pose = self.init_pose.pose

        # Beacon Pose Estimation
        self.neighbors = dict()
        self.beacon_estimates = dict()
        self.beacon_est_pose = PoseStamped()
        
        # Sensor Data
        self.water_pressure = FluidPressure()
        self.imu = Imu()
        self.gps = NavSatFix()

        # Commands
        self.prev_cmd = UwGliderCommand()
        self.cmd = UwGliderCommand()

        self.pitch_mode = 3    # [Pitch] Desired Angle with servo compensation (radians)
        self.rudder_mode = 1   # [Yaw] Rudder Angle at the center (radians)
        self.motor_mode = 1    # Voltage [0-100]

        self.desired_yaw_angle = 90
        self.desired_pitch_angle = 20
        self.pump_val = 0      # Neutral Buoyancy at 0; Positive ascend

        # REMOVE THESE LATER
        self.yaw_in_deg = 90
        self.time0 = rospy.Time.now()

        # Logging stuff
        self.grapher = Graphing3D(include_time=True)
        #self.grapher.add_path(AUV_TRUE_POSE_1, "Ground Truth")
        #self.grapher.add_path(AUV_EST_POSE_1, "Dead Reckoning")
        #self.grapher.add_path(AUV_BEACON_POSE_1, "Beacon Localization: Average")
        #self.grapher.add_path(AUV_BEACON_POSE_2, "Beacon Localization: Weighted Average")
        #self.grapher.add_path(AUV_BEACON_POSE_3, "Beacon Localization: Closest Neighbor")
        self.log_stamp = self.grapher.init_paths(labels)
        self.t_pose_stamp = 0

        # Temporary
        self.start_time = 0
        self.prev_start_time = 0

    def __true_pose_cbk(self, data):
        if data is not None:
            self.true_pose = data
            t_pos = self.true_pose.pose.position
            t_stamp = self.true_pose.header.stamp.secs
            if self.t_pose_stamp != t_stamp: 
                self.grapher.send_data_to_csv(AUV_TRUE_POSE_1, self.log_stamp, t_pos.x, t_pos.y, t_pos.z, self.true_pose.header.stamp.secs)
                self.t_pose_stamp = t_stamp
            # self.grapher.add_path_point(AUV_TRUE_POSE_1, t_pos.x, t_pos.y, t_pos.z, self.true_pose.header.stamp.secs)
            # rospy.loginfo(f"True Pose: ({t_pos.x}, {t_pos.y}, {t_pos.z})")
    
    def __status_cbk(self, data):
        self.uwg_status = data
        # rospy.loginfo(self.status_msg(self.uwg_status))

    def __comm_cbk(self, msg):
        if msg is not None:
            msg_str = msg.data
            if msg_str != 'ping':
                msg_obj = json.loads(msg_str)
                bid = msg_obj["bid"]
                
                if bid not in self.neighbors.keys():        
                    self.neighbors[bid] = PoseStamped()
            
                del msg_obj["bid"]
                msg_str = json.dumps(msg_obj) 
                pos_msg = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseStamped', msg_str)
                self.neighbors[bid] = pos_msg #strm.rosmsg_from_str(msg_str)
        
                """rospy.loginfo("--------------------------------")
                for bid, node in self.neighbors.items():
                    rospy.loginfo(f"{bid}: ({node.pose.position.x}, {node.pose.position.y}, {node.pose.position.z})")
                rospy.loginfo("--------------------------------")"""

    def __pressure_cbk(self, data):
        self.water_pressure = data
        #self.depth_estimate = self.calc_depth()
        # rospy.loginfo(f"Depth Estimate: {self.depth_estimate}, Actual Depth: {self.true_pose.pose.position.z}")

    def __imu_cbk(self, data):
        self.imu = data
        self.update_pose_estimate(self.aoa_deg)

    def __gps_cbk(self, data):
        try:        
            self.gps = data
        except:
            pass

    # Averaging position estimation from neighboring beacons
    def __loc_cbk(self, data):
        if data is not None:
            #print(data)
            #self.beacon_estimates.append(data)
            pass

    def __resp_cbk(self, data):
        if data is not None:
            pass
    
    def __get_distance(self,curr_pose, next_pose):
        sum_of_squares = math.pow(curr_pose.position.x - next_pose.position.x, 2) + \
                         math.pow(curr_pose.position.y - next_pose.position.y, 2) + \
                         math.pow(curr_pose.position.z - next_pose.position.z, 2)
        dist = math.sqrt(math.fabs(sum_of_squares))
        return dist

    def get_path(self):
        path = Path()
        for pt in WAYPOINTS:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = pt[0]
            tmp_pose.pose.position.y = pt[1]
            tmp_pose.pose.position.z = pt[2]
            path.poses.append(tmp_pose)
        return path

    def calc_depth(self):
        depth = 0
        try:
            standardPressure = 101.325                    # data found in glider_hybrid_whoi/glidereckoining/nodes/deadreckoning_estimator.py 
            KPaPerM = 9.80838
            depth = -round((self.water_pressure.fluid_pressure - standardPressure)/KPaPerM, 2)
            return depth
            # rospy.loginfo(f"Depth Estimate: {self.depth_estimate}, Actual Depth: {self.true_pose.pose.position.z}")
        except:
            return None

    def status_msg(self, status):
        return f"""
        Roll:        {status.roll}
        Pitch:       {status.pitch}
        Heading:     {status.heading}
        Depth:       {status.depth}
        Motor Power: {status.motor_power}
        Pump Volume: {status.pumped_volume}
        """

    """def init_app(self):
        from sys import argv
        UWGC_TOPIC = f'/{argv[1]}/kinematics/UwGliderCommand'
        UWGS_TOPIC = f'/{argv[1]}/kinematics/UwGliderStatus'
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose'
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'

        # Transciever Topics
        RESPONSE_TOPIC = "/USBL/transceiver_manufacturer_168/command_response"
        INTERROGATION_TOPIC = '/USBL/transceiver_manufacturer_168/interrogation_mode'
        CHANNEL_SWITCH_TOPIC = '/USBL/transceiver_manufacturer_168/channel_switch'
        LOCATION_TOPIC = "/USBL/transceiver_manufacturer_168/transponder_location_cartesion"

        # Transponder Topics
        REQUEST_TOPIC = "/USBL/transponder_manufacturer_1/command_request"
        COMMON_TOPIC = '/USBL/common_interrogation_ping'
        INDIVIDUAL_TOPIC = '/USBL/transponder_manufacturer_1/individual_interrogation_ping'

        #Initialize Publishers
        self.command_pub = rospy.Publisher(UWGC_TOPIC, UwGliderCommand, queue_size=1)
        self.ch_switch_pub = rospy.Publisher(CHANNEL_SWITCH_TOPIC, String, queue_size=1)
 
        #Initializee Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(UWGS_TOPIC, UwGliderStatus, self.__status_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)
        rospy.Subscriber(COMMON_TOPIC, String, self.__comm_cbk)
        rospy.Subscriber(LOCATION_TOPIC, Vector3, self.__loc_cbk)
        """

    def init_app(self):
        from sys import argv
        UWGC_TOPIC = f'/{argv[1]}/kinematics/UwGliderCommand'
        UWGS_TOPIC = f'/{argv[1]}/kinematics/UwGliderStatus'
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose'
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'
        POSE_LOG_TOPIC = f'/{argv[1]}/logging/pose'

        # Communication topics
        COMMON_TOPIC = "/USBL/common_ping"
        LOCATION_TOPIC = f"/USBL/transceiver_{self.transceiver_id}/transponder_pose"

        RESP_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'

        #self.rx = Transponder(self.transponder_id, self.transceiver_id, argv[1])
        # timestamp = datetime.now().isoformat('_', timespec='seconds')
        self.logger = GliderLogger(argv[1], self.log_stamp, POSE_LOG_TOPIC)
        self.logger.start()

        #Initialize Publishers
        self.command_pub = rospy.Publisher(UWGC_TOPIC, UwGliderCommand, queue_size=1)
        self.pose_log_pub = rospy.Publisher(POSE_LOG_TOPIC, VehicleLog, queue_size=1)
 
        #Initializee Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(UWGS_TOPIC, UwGliderStatus, self.__status_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)
        rospy.Subscriber(COMMON_TOPIC, String, self.__comm_cbk)
        rospy.Subscriber(RESP_TOPIC, USBLResponseSim, self.__resp_cbk)
        self.loc_sub = rospy.Subscriber(LOCATION_TOPIC, PoseStamped, self.__loc_cbk)

    ########################################
    # Communication/Localization Functions #
    ########################################
    
    """def ping_neigbors(self):
        msg_str = json.dumps(INDIVIDUAL_MSG)
        self.ch_switch_pub.publish(msg_str)

        for bid in self.neighbors.keys():
            NEIGHBOR_TOPIC = f"/USBL/transponder_manufacturer_{bid}/individual_interrogation_ping"
            hello_pub = rospy.Publisher(NEIGHBOR_TOPIC, String, queue_size=1)
            hello_str = json.dumps(HELLO_MSG)
            hello_pub.publish(hello_str)
    """

    def query_beacons(self):
        temp_dict = self.neighbors.copy()
        for bid in temp_dict.keys():
            time.sleep(1)
            self.rx.set_tx_channel(bid)
            self.rx.send_location_request()
            resp = self.rx.get_response()
            try:
                msg_str = resp.data
                loc = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseStamped', msg_str)
                self.beacon_estimates[bid] = loc
                #print(self.beacon_estimates[bid].pose.position)
                time.sleep(3)
            except Exception as e:
                print(e)

    def get_simple_average(self):
        x = 0
        y = 0
        z = 0
        beacon_num = 0
        for bid in self.beacon_estimates.keys():
            try:
                node = self.beacon_estimates[bid]
                # rospy.loginfo(f"Beacon {bid}: ({node.pose.position.x}, {node.pose.position.y}, {node.pose.position.z})")
                x += node.pose.position.x
                y += node.pose.position.y
                z += node.pose.position.z
                beacon_num += 1
            except:
                pass
        if beacon_num > 0:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = x/beacon_num
            tmp_pose.pose.position.y = y/beacon_num
            tmp_pose.pose.position.z = z/beacon_num
            return tmp_pose
        return None
    
    def get_weighted_average_by_time(self):
        x, y, z, weight_num = 0, 0, 0, 0
        curr_time = rospy.Time.now()
        time_thres = 60
        for bid in self.beacon_estimates.keys():
            try:
                node = self.beacon_estimates[bid]
                time_interval = 15
                # Reward estimations based on transmission time in seconds
                weight = round(9*(time_interval/(curr_time.secs - node.header.stamp.secs)), 2)
                #rospy.loginfo(f"Time Delay: {curr_time - node.header.stamp.to_sec()}, Weight: {weight}")
                #rospy.loginfo(f"Beacon {bid}: ({node.pose.position.x}, {node.pose.position.y}, {node.pose.position.z})")
                x += node.pose.position.x * weight
                y += node.pose.position.y * weight
                z += node.pose.position.z * weight
                weight_num += weight
            except Exception as e:
                print(e)
        if weight_num > 0:
            tmp_pose = PoseStamped()
            tmp_pose.header.stamp = curr_time
            tmp_pose.pose.position.x = x/weight_num
            tmp_pose.pose.position.y = y/weight_num
            tmp_pose.pose.position.z = z/weight_num
            return tmp_pose
        return None
    
    def get_closest_neighbor(self):
        try:
            closest_dist = float("inf")
            beacon_id = -1
            for bid in self.beacon_estimates.keys():
                e_pos = self.beacon_estimates[bid]
                b_pos = self.neighbors[bid]
                dist = self.__get_distance(b_pos.pose, e_pos.pose)
                #rospy.loginfo(f"Beacon {bid}: ({e_pos.pose.position.x}, {e_pos.pose.position.y}, {e_pos.pose.position.z})")
                if dist < closest_dist:
                    closest_dist = dist
                    beacon_id = bid
            self.transceiver_id = f'{beacon_id}'
        except:
            rospy.loginfo("Neighbors could not be found.")

    def get_pose_from_closest_neighbor(self):
        self.get_closest_neighbor()
        if self.transceiver_id in self.beacon_estimates.keys():
            return self.beacon_estimates[self.transceiver_id]
        else:
            None

    def get_pose_by_closest_neighbor(self):
        curr_time = rospy.Time.now().to_sec()
        time_thres = 60
        closest_dist = float("inf")
        tmp_pose = None
        for bid in self.beacon_estimates.keys():
            try:
                e_pos = self.beacon_estimates[bid]
                b_pos = self.neighbors[bid]
                dist = self.__get_distance(b_pos.pose, e_pos.pose)
                #rospy.loginfo(f"Beacon {bid}: ({e_pos.pose.position.x}, {e_pos.pose.position.y}, {e_pos.pose.position.z})")
                if dist < closest_dist:
                    closest_dist = dist
                    tmp_pose = e_pos
                
            except Exception as e:
                print(e)
        return tmp_pose
    
    def update_pose_estimate(self, aoa_rad):
        orient = self.imu.orientation
        row, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        # rospy.loginfo(f"RPY: ({row}, {pitch}, {yaw})")

        curr_time = rospy.Time.now()
        dt = curr_time.secs - self.time0.secs
        if dt < 1.0e-3:
            return
        self.time0 = curr_time

        depth = self.calc_depth()
        dz = depth - self.prev_depth 
        self.prev_depth = depth

        if pitch < 0:
            glide_angle = pitch - aoa_rad
        else:
            glide_angle = pitch + aoa_rad
        # rospy.loginfo(f"Dz: {dz}, Glide Angle: {glide_angle}")

        if abs(pitch) < math.pi/100.0:
            v_x = 0.0
            v_y = 0.0
        else:
            #deg = yaw * (180/math.pi)
            v_x = dz/(dt*math.tan(glide_angle))*math.sin(math.pi/2.0-yaw)
            v_y = dz/(dt*math.tan(glide_angle))*math.cos(math.pi/2.0-yaw)
        d_x = v_x * dt
        d_y = v_y * dt
        #rospy.loginfo(f"vx: {v_x}, dx: {d_x}, vx: {v_y}, dx: {d_y}")

        #if self.est_pose is None:
        #    self.est_pose = self.init_pose
        #else:
        self.est_pose.header.stamp = rospy.Time.now()
        self.est_pose.pose.position.x += d_x #/math.sin(yaw*2.0*math.pi/360)
        self.est_pose.pose.position.y += d_y #/math.cos(yaw*2.0*math.pi/360)
        self.est_pose.pose.position.z = depth
        self.est_pose.pose.orientation.x = self.imu.orientation.x
        self.est_pose.pose.orientation.y = self.imu.orientation.y
        self.est_pose.pose.orientation.z = self.imu.orientation.z
        self.est_pose.pose.orientation.w = self.imu.orientation.w

        e_pos = self.est_pose.pose.position
        t_pos = self.true_pose.pose.position

        if self.est_pose is not None and self.imu is not None:
            vl = VehicleLog()
            vl.header = self.imu.header
            vl.position = self.est_pose.pose.position
            vl.orientation = self.imu.orientation
            vl.orientation_covariance = self.imu.orientation_covariance
            vl.angular_velocity = self.imu.angular_velocity
            vl.angular_velocity_covariance = self.imu.angular_velocity_covariance
            vl.linear_acceleration = self.imu.linear_acceleration
            vl.linear_acceleration_covariance = self.imu.linear_acceleration_covariance
            self.pose_log_pub.publish(vl)
            self.grapher.send_data_to_csv(AUV_EST_POSE_1, self.log_stamp, self.est_pose.pose.position.x, self.est_pose.pose.position.y, self.est_pose.pose.position.z, self.est_pose.header.stamp.secs)
            #self.grapher.add_path_point(AUV_EST_POSE_1, e_pos.x, e_pos.y, e_pos.z, self.est_pose.header.stamp.secs)
        return self.est_pose if not None else PoseStamped()
    

    def get_new_pose_from_logs(self, data, starting_pose):
        final_pose = PoseStamped()
        final_pose.pose.position.x = starting_pose.pose.position.x
        final_pose.pose.position.y = starting_pose.pose.position.y
        final_pose.pose.position.z = starting_pose.pose.position.z

        final_pose.pose.orientation.x = starting_pose.pose.orientation.x
        final_pose.pose.orientation.y = starting_pose.pose.orientation.y
        final_pose.pose.orientation.z = starting_pose.pose.orientation.z
        final_pose.pose.orientation.w = starting_pose.pose.orientation.w 

        prev_depth = 0
        prev_time = 0
        aoa_deg = -0.35

        for val in data:
            rpy = val[10:]
            if float(rpy[1]) < 0:
                glide_angle = float(rpy[1]) + aoa_deg
            else:
                glide_angle = float(rpy[1]) + aoa_deg
            
            depth = float(val[2])
            dz = depth - prev_depth 
            prev_depth = depth

            time = int(val[0])
            dt = time - prev_time
            prev_time = time

            if abs(float(rpy[1])) < math.pi/100.0:
                v_x = 0.0
                v_y = 0.0
            else:
                #deg = yaw * (180/math.pi)
                v_x = dz/(dt*math.tan(glide_angle))*math.sin(math.pi/2.0-float(rpy[2]))
                v_y = dz/(dt*math.tan(glide_angle))*math.cos(math.pi/2.0-float(rpy[2]))
            d_x = v_x * dt
            d_y = v_y * dt
            final_pose.header.stamp.secs = int(val[0])
            final_pose.pose.position.x -= d_x
            final_pose.pose.position.y -= d_y
        final_pose.pose.position.z = self.calc_depth()
        #rospy.loginfo(f"\n Positon:\n{starting_pose.pose.position}\n New Position:\n{final_pose.pose.position}")
        return final_pose

    def w_avg_dead_reckoning(self):
        self.get_closest_neighbor()
        wavg_pose = self.get_weighted_average_by_time()
        if wavg_pose is not None:
            try:
                start_time = self.start_time
                if start_time != self.prev_start_time:
                    start_time = self.beacon_estimates[self.transceiver_id].header.stamp.secs
                end_time = rospy.Time.now().secs
                print(f"Time span: {start_time} to {end_time}")
                data = self.logger.retrieve_data_by_time(start_time, end_time)
                fuse_pose = self.get_new_pose_from_logs(data, wavg_pose)
                self.prev_start_time = self.start_time
                self.start_time = end_time
                return fuse_pose
            except:
                pass
            #if self.start_time <= wavg_pose.header.stamp.secs:
            #    start_time = wavg_pose.header.stamp.secs
            #elif self.start_time > wavg_pose.header.stamp.secs:
            #start_time = self.start_time
            
        return None


    ##############
    # NAVIGATION #
    ##############

    def get_path_idx(self, path, vehicle_pose):
        idx = 0
        closest_pt = None                                    # Store closest point for reference
        closest_pt_dist = float("inf")    
        for i in range(len(path.poses)-1):
            if closest_pt is None:
                closest_pt = path.poses[i].pose
                closest_pt_dist = self.__get_distance(vehicle_pose.pose, closest_pt)
                idx = i
            else:
                curr_pt = path.poses[i].pose
                dist = self.__get_distance(vehicle_pose.pose, curr_pt)
                if dist < closest_pt_dist:
                    closest_pt_dist = curr_pt
                    closest_pt_dist = dist
                    idx = i
            if closest_pt_dist < 5:
                idx += 1
        return idx

    def get_desired_values(self, vehicle_pose, current_goal):
        orient = vehicle_pose.pose.orientation
        curr_roll, curr_pitch, curr_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        desired_yaw = 0
        desired_pitch = 0 
        desired_tank = 0

        desired_yaw = math.atan2((current_goal.pose.position.y-vehicle_pose.pose.position.y), (current_goal.pose.position.x-vehicle_pose.pose.position.x))
        desired_pitch = math.atan2((current_goal.pose.position.z-vehicle_pose.pose.position.z),(current_goal.pose.position.x-vehicle_pose.pose.position.x))
        
        yaw_angle = desired_yaw - curr_yaw
        pitch_angle = desired_pitch - curr_pitch

        # Pitching down
        #if desired_pitch > 0:
        #    desired_tank = 0.4
        # Pitching up
        #elif desired_pitch < 0:
        #    desired_tank = -0.2
        depth = self.calc_depth()
        offset = 2
        diff = round(0.15*((current_goal.pose.position.z + offset) - depth), 2)
        if diff > 0.4:
                desired_pitch = 0.4
        elif diff < -0.4:
                desired_pitch = -0.4
        else:
            desired_pitch = diff

        if math.fabs(current_goal.pose.position.z - depth) < 0.3:
                desired_tank = 0
        elif depth < current_goal.pose.position.z:
                desired_tank = -80
        elif depth > current_goal.pose.position.z:
                desired_tank = 80
        
        # rospy.loginfo(f"Desired Yaw: {desired_yaw}, Desired Pitch: {desired_pitch}, Desired Tank: {desired_tank}")
        return desired_yaw, 0, desired_tank 

    def command_vehicle(self, desired_heading, desired_pitch, pump_val):
        
        heading_val = desired_heading*(math.pi/180.0)  
        pitch_val = desired_pitch*(math.pi/180.0)
        motor_val = 10 # Threshold: [0-100]

        # Limits 
        if heading_val < YAW_LOWER_LIMIT:
            heading_val = YAW_LOWER_LIMIT
        elif heading_val > YAW_UPPER_LIMIT:
            heading_val = YAW_UPPER_LIMIT
        
        if pitch_val < PITCH_LOWER_LIMIT:
            pitch_val = PITCH_LOWER_LIMIT
        elif pitch_val > PITCH_UPPER_LIMIT:
            pitch_val = PITCH_UPPER_LIMIT

        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = self.pitch_mode
        self.cmd.target_pitch_value = pitch_val
        self.cmd.target_pumped_volume = pump_val
        self.cmd.rudder_control_mode = self.rudder_mode
        self.cmd.target_heading = heading_val
        self.cmd.motor_cmd_type = self.motor_mode
        self.cmd.target_motor_cmd = motor_val
        self.command_pub.publish(self.cmd)

    def stop(self):
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.target_motor_cmd = 0
        self.command_pub.publish(self.cmd)

    def circle(self):
        self.yaw_in_deg -= 1 
        if self.yaw_in_deg < 0:
            self.yaw_in_deg = 360
        elif self.yaw_in_deg > 360:
            self.yaw_in_deg = 0

        yaw_in_rad = self.yaw_in_deg*2.0*math.pi/360.0
        est_pose0 = self.update_pose_estimate(self.aoa_deg)

        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.pitch_cmd_type = 1
        self.cmd.target_pitch_value = 0.4
        self.cmd.target_pumped_volume = 50 #100.0
        self.cmd.rudder_control_mode = 1
        self.cmd.target_heading = self.yaw_in_deg #160 #4.0*math.pi/9.0
        self.cmd.motor_cmd_type = 1
        self.cmd.target_motor_cmd = 20.0
        self.command_pub.publish(self.cmd)

    def circle_test(self, start_time):
        curr_time = rospy.Time.now()
        if curr_time.secs - start_time.secs < 61:
            self.query_beacons()
            #est_pose1 = self.get_simple_average()
            est_pose2 = self.get_weighted_average_by_time()
            #est_pose3 = self.get_pose_from_closest_neighbor()
            f_pose = self.w_avg_dead_reckoning()
            if f_pose is not None:
                self.grapher.send_data_to_csv(AUV_FUSION_POSE_2, self.log_stamp, f_pose.pose.position.x, f_pose.pose.position.y, f_pose.pose.position.z, f_pose.header.stamp.secs)
            """if est_pose1 is not None:
                self.grapher.send_data_to_csv(AUV_BEACON_POSE_1, self.log_stamp, est_pose1.pose.position.x, est_pose1.pose.position.y, est_pose1.pose.position.z, curr_time.secs)
                # self.grapher.add_path_point(AUV_BEACON_POSE_1, est_pose1.pose.position.x, est_pose1.pose.position.y, est_pose1.pose.position.z, curr_time.secs)"""
            if est_pose2 is not None:
                self.grapher.send_data_to_csv(AUV_BEACON_POSE_2, self.log_stamp, est_pose2.pose.position.x, est_pose2.pose.position.y, est_pose2.pose.position.z, curr_time.secs)
                # self.grapher.add_path_point(AUV_BEACON_POSE_2, est_pose2.pose.position.x, est_pose2.pose.position.y, est_pose2.pose.position.z, curr_time.secs)
            """if est_pose3 is not None:
                self.grapher.send_data_to_csv(AUV_BEACON_POSE_3, self.log_stamp, est_pose3.pose.position.x, est_pose3.pose.position.y, est_pose3.pose.position.z, curr_time.secs)
                # self.grapher.add_path_point(AUV_BEACON_POSE_3, est_pose3.pose.position.x, est_pose3.pose.position.y, est_pose3.pose.position.z, curr_time.secs)"""
            self.circle()
            time.sleep(2)
        else:
            self.stop()
            # self.grapher.show_plot("AUV Helical Test")
            #self.grapher.graph_data_from_csv(labels, self.log_stamp, "Helical Glide Test")
            rospy.signal_shutdown("Task Complete")

    def run(self):
        """path = self.get_path()
        path_complete = False

        goal_pose = Pose()
        if len(path.poses) >= 1:
            goal_pose = path.poses[len(path.poses)-1].pose"""

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            self.circle_test(start_time)
            """if len(path.poses) != 0:
                path_idx = self.get_path_idx(path, self.true_pose)
                current_goal = path.poses[path_idx]
                #rospy.loginfo("---------------------------------------------------------------------------------")
                #rospy.loginfo(f"Current position: ({self.true_pose.pose.position.x}, {self.true_pose.pose.position.y}, {self.true_pose.pose.position.z})")
                #rospy.loginfo(f"Current goal: ({current_goal.pose.position.x}, {current_goal.pose.position.y}, {current_goal.pose.position.z})")
                desired_heading, desired_pitch, tank_val = self.get_desired_values(self.true_pose, current_goal)
                self.command_vehicle(desired_heading, desired_pitch, tank_val)
                #rospy.loginfo(self.status_msg(self.uwg_status))
                #rospy.loginfo("---------------------------------------------------------------------------------")
                if math.fabs(self.true_pose.pose.position.x - goal_pose.position.x) <= COMPLETION_THRESHOLD and \
                   math.fabs(self.true_pose.pose.position.y - goal_pose.position.y) <= COMPLETION_THRESHOLD and \
                   math.fabs(self.true_pose.pose.position.y - goal_pose.position.y) <= COMPLETION_THRESHOLD:
                    path_complete = True
            if path_complete == True:
                self.grapher.show_plot("Waypoint Navigation")
                rospy.signal_shutdown("Path Complete")"""
            self.rate.sleep() 

if __name__ == "__main__":
    from sys import argv
    rospy.init_node(f"{argv[1]}_wnav")
    wf = WaypointFollower()
    wf.init_app()
    try:
        """waypts = "waypoints"
        wf.grapher.add_path(waypts,"Waypoints")
        for point in WAYPOINTS:
            wf.grapher.add_path_point(waypts, point[0], point[1], point[2])"""
        wf.run()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{argv[1]} waypoint navigation terminated.")