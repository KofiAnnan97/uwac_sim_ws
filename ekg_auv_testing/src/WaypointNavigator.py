#!/usr/bin/env python3

import rospy
import math
import time
import json

from frl_vehicle_msgs.msg import UwGliderCommand, UwGliderStatus
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from StringtoROSmsg import StringtoROSmsg
from rospy_message_converter import message_converter, json_message_converter

from Graphing3D import Graphing3D
from TestWaypointPaths import get_circle_path, get_helix_path, get_double_helix_path
from USBL import Transponder
from ekg_auv_testing.msg import USBLRequestSim, USBLResponseSim

AUV_TRUE_POSE_1 = "true_pose"
AUV_EST_POSE_1 = "estimate_pose"
AUV_BEACON_POSE_1 = "beacon_pose"

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

        self.grapher = Graphing3D()
        self.grapher.add_path(AUV_TRUE_POSE_1, "True Pose AUV 1")
        self.grapher.add_path(AUV_EST_POSE_1, "Estimated Pose AUV 1")

        #Ground Truth
        self.true_pose = PoseStamped()
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])
        self.uwg_status = UwGliderStatus()

        self.aoa_deg = -3.0
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

    def __true_pose_cbk(self, data):
        self.true_pose = data
        t_pos = self.true_pose.pose.position 
        #self.grapher.add_path_point(AUV_TRUE_POSE_1, t_pos.x, t_pos.y, t_pos.z)
        #rospy.loginfo(f"True Pose: ({t_pos.x}, {t_pos.y}, {t_pos.z})")
    
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
        #self.update_pose_estimate(self.aoa_deg*math.pi/180)

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
        
    def get_pose_average(self):
        x = 0
        y = 0
        z = 0
        for node in self.beacon_estimates:
            try:
                x += node.pose.position.x
                y += node.pose.position.y
                z += node.pose.position.z
            except:
                pass
        beacon_num = len(self.beacon_estimates)
        if beacon_num > 0:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = x/beacon_num
            tmp_pose.pose.position.y = y/beacon_num
            tmp_pose.pose.position.z = z/beacon_num
            return tmp_pose
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

    def init_app(self):
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

    def init_app2(self):
        from sys import argv
        UWGC_TOPIC = f'/{argv[1]}/kinematics/UwGliderCommand'
        UWGS_TOPIC = f'/{argv[1]}/kinematics/UwGliderStatus'
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose'
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'

        # Communication topics
        COMMON_TOPIC = "/USBL/common_ping"
        LOCATION_TOPIC = f"/USBL/transceiver_{self.transceiver_id}/transponder_pose"

        RESP_TOPIC = f'/USBL/transponder_{self.transponder_id}/command_response'

        self.rx = Transponder(self.transponder_id, self.transceiver_id, argv[1])

        #Initialize Publishers
        self.command_pub = rospy.Publisher(UWGC_TOPIC, UwGliderCommand, queue_size=1)
 
        #Initializee Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(UWGS_TOPIC, UwGliderStatus, self.__status_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)
        rospy.Subscriber(COMMON_TOPIC, String, self.__comm_cbk)
        rospy.Subscriber(RESP_TOPIC, USBLResponseSim, self.__resp_cbk)
        self.loc_sub = rospy.Subscriber(LOCATION_TOPIC, PoseStamped, self.__loc_cbk)

    def ping_neigbors(self):
        msg_str = json.dumps(INDIVIDUAL_MSG)
        self.ch_switch_pub.publish(msg_str)

        for bid in self.neighbors.keys():
            NEIGHBOR_TOPIC = f"/USBL/transponder_manufacturer_{bid}/individual_interrogation_ping"
            hello_pub = rospy.Publisher(NEIGHBOR_TOPIC, String, queue_size=1)
            hello_str = json.dumps(HELLO_MSG)
            hello_pub.publish(hello_str)

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
            except Exception as e:
                print(e)

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

        if self.est_pose is None:
            self.est_pose = self.init_pose
        else:
            self.est_pose.header.stamp = rospy.Time.now()
            self.est_pose.pose.position.x += d_x #/math.sin(yaw*2.0*math.pi/360)
            self.est_pose.pose.position.y += d_y #/math.cos(yaw*2.0*math.pi/360)
            self.est_pose.pose.position.z = depth
        e_pos = self.est_pose.pose.position
        t_pos = self.true_pose.pose.position
        self.grapher.add_path_point(AUV_TRUE_POSE_1, t_pos.x, t_pos.y, t_pos.z)
        self.grapher.add_path_point(AUV_EST_POSE_1, e_pos.x, e_pos.y, e_pos.z)
        #rospy.loginfo(f"Current position: ({t_pos.x}, {t_pos.y}, {t_pos.z})")
        #rospy.loginfo(f"Estimated position: ({e_pos.x}, {e_pos.y}, {e_pos.z})")
        return self.est_pose if not None else PoseStamped()

    """def get_path_idx(self, path, vehicle_pose):
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
    """

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
        self.update_pose_estimate(self.aoa_deg)

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
        #if curr_time.secs - start_time.secs < 40000: #30
        self.query_beacons()
        self.circle()
        time.sleep(2)
        #else:
        #    self.stop()
        #    self.grapher.show_plot("AUV Helical Test")
        #    rospy.signal_shutdown("Task Complete")

    def run(self):
        path = self.get_path()
        path_complete = False

        goal_pose = Pose()
        if len(path.poses) >= 1:
            goal_pose = path.poses[len(path.poses)-1].pose

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
    wf.init_app2()
    try:
        """waypts = "waypoints"
        wf.grapher.add_path(waypts,"Waypoints")
        for point in WAYPOINTS:
            wf.grapher.add_path_point(waypts, point[0], point[1], point[2])"""
        wf.run()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{argv[1]} waypoint navigation terminated.")