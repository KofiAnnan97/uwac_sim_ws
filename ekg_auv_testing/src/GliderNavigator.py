# !/usr/bin/env python3

import rospy
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist, PoseWithCovariance, Quaternion
from nav_msgs.msg import Path
from frl_vehicle_msgs.msg import UwGliderCommand, UwGliderStatus
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix

WAYPOINTS = [
    (4, 4, -18.5),
    (5, 5, -18.5),
    (6, 6, -18.5),
    (7, 8, -18.5),
    (9, 9, -18.5),
    (10, 20, -18.5),
    (3, 15, -18.5)
]

class GliderNavigator:
    def __init__(self) -> None:
        self.init_pose = PoseStamped()
        from sys import argv
        self.init_pose.pose.position.x = float(argv[2])
        self.init_pose.pose.position.y = float(argv[3])
        self.init_pose.pose.position.z = float(argv[4])

        self.curr_pose = PoseStamped()
        self.path = Path()
    
    def init_app(self):
        from sys import argv
        UWGC_TOPIC = f'/{argv[1]}/kinematics/UwGliderCommand'
        UWGS_TOPIC = f'/{argv[1]}/kinematics/UwGliderStatus'
        TRUE_POSE_TOPIC = f'/{argv[1]}/ground_truth_to_tf_{argv[1]}/pose'
        IMU_TOPIC = f'/{argv[1]}/hector_imu'
        PRESSURE_TOPIC = f'/{argv[1]}/pressure'
        GPS_TOPIC = f'/{argv[1]}/hector_gps'

        self.rate = rospy.Rate(10)

        # Publishers
        self.command_pub = rospy.Publisher(UWGC_TOPIC, UwGliderCommand, queue_size=1)
        # Subscribers
        rospy.Subscriber(TRUE_POSE_TOPIC, PoseStamped, self.__true_pose_cbk)
        rospy.Subscriber(UWGS_TOPIC, UwGliderStatus, self.__status_cbk)
        rospy.Subscriber(PRESSURE_TOPIC, FluidPressure, self.__pressure_cbk)
        rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cbk)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.__gps_cbk)

    def __true_pose_cbk(self, data):
        self.true_pose = data
        # t_pos = self.true_pose.pose.position 
        # self.grapher.add_path_point(AUV_TRUE_POSE_1, t_pos.x, t_pos.y, t_pos.z)
    
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

    def __get_distance_btw_pts(self, curr_pose, next_pose):
        return math.sqrt(pow(next_pose.position.x - curr_pose.position.x, 2) + \
                        pow(next_pose.position.y - curr_pose.position.y, 2) + \
                        pow(next_pose.position.z - curr_pose.position.z, 2))

    def static_path_planner(self,waypoints):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        for pt in waypoints:
            tmp_pose = PoseStamped()
            tmp_pose.header.stamp = rospy.Time.now()
            tmp_pose.header.frame_id = 'map'
            tmp_pose.pose.position.x = pt[0]
            tmp_pose.pose.position.y = pt[1]
            tmp_pose.pose.position.z = pt[2]
            path.poses.append(tmp_pose)
        return path
    def get_path_idx(self,path,vehicle_pose):
        idx = 0
        closest_pt = None                                    # Store closest point for reference
        closest_pt_dist = float("inf")                       # Store closest point distance for reference
        for i in range(len(path.poses)-1):
            curr_pt = path.poses[i].pose
            #print(f"{closest_pt}\n{curr_pt.position}")
            dist = self.__get_distance_btw_pts(vehicle_pose.pose, curr_pt)
            #print(f"Closed distance: {closest_pt_dist}, Current Pt distance: {dist}")
            if dist < closest_pt_dist:
                closest_pt_dist = curr_pt
                closest_pt_dist = dist
                idx = i
        if len(path.poses) > 1:
            #idx+=1                                         # Increment to next point
            #print(f"Current idx: {idx}\nCurrent goal: ({path.poses[idx].pose.position.x}, {path.poses[idx].pose.position.y})") 
            for j in range(idx, len(path.poses)-1):
                curr_goal = path.poses[idx].pose
                next_pose = path.poses[j].pose
                if curr_goal.position.x == next_pose.position.x or \
                   curr_goal.position.y == next_pose.position.y:
                    idx = j
                else:
                    break
        #print(f"New idx: {idx}\nNew goal: ({path.poses[idx].pose.position.x}, {path.poses[idx].pose.position.y})")
        #print(f"Vehicle: ({vehicle_pose.pose.position.x}, {vehicle_pose.pose.position.y})") 
        #print(f"Closest Pt: ({closest_pt.position.x},{closest_pt.position.y}) Dist: {closest_pt_dist}")
        return idx

    def path_follower(self,vehicle_pose, current_goal_pose):
        speed = 0.1
        heading = 0
        pitch = 0
        #rospy.loginfo(f"\nVEHICLE: ({vehicle_pose.pose.position.x}, {vehicle_pose.pose.position.y})\nGOAL: ({current_goal_pose.position.x}, {current_goal_pose.position.y})")
        g_pos = current_goal_pose.pose.position
        v_pos = vehicle_pose.pose.position
        v_dir = vehicle_pose.pose.orientation
        curr_roll, curr_pitch, curr_yaw = euler_from_quaternion([v_dir.x, v_dir.y, v_dir.z, v_dir.w])
        desired_orientation = Quaternion()
        try:    
            dist = self.__get_distance_btw_pts(vehicle_pose.pose, current_goal_pose.pose)
            desired_yaw = math.atan2((g_pos.y - v_pos.y), (g_pos.x - v_pos.x))
            desired_pitch = math.atan2((g_pos.y - v_pos.y), dist - pow(g_pos.y - v_pos.y, 2))

            tmp_orientation = quaternion_from_euler(curr_roll, desired_pitch, desired_yaw)
            desired_orientation.x = tmp_orientation[0]
            desired_orientation.y = tmp_orientation[1]
            desired_orientation.z = tmp_orientation[2]
            desired_orientation.w = tmp_orientation[3]
            heading =  desired_yaw - curr_yaw
            pitch = desired_pitch - curr_pitch
        except:
            rospy.logerr("Desired angle could not be determined. Zero may be the denominator.")
        speed = abs(dist)
        rospy.loginfo(f"\nCurrent Pitch: {curr_pitch}\nDesired Pitch: {desired_pitch}\nCurrent Yaw: {curr_yaw}\nDesired Yaw: {desired_yaw}\n---------------------------")
        return speed,heading, pitch

    def send_command(self):
        glider_cmd = UwGliderCommand()
        self.command_pub(glider_cmd)


    def run(self):
        #last_goal = self.goal_pose
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        #path_complete = False
        #timeout = False
        path = self.static_path_planner(WAYPOINTS)
        #print(path)

        while not rospy.is_shutdown():
            # 1. Create the path to follow
            # Check to see if new goal has been given 
            #if self.goal_pose.pose.position.x != last_goal.pose.position.x or \
            #   self.goal_pose.pose.position.y != last_goal.pose.position.y:
                #rospy.loginfo(f"\n== LAST GOAL ==\n{last_goal.pose.position}\n== CURRENT GOAL ==\n{self.goal_pose.pose.position}")
            
            #    self.path_pub.publish(path)
                #self.print_path_to_file(path)
                #last_goal = self.goal_pose
            # 2. Loop through the path and move the robot
            if len(path.poses) > 0:
                idx = self.get_path_idx(path,self.ttbot_pose)
                current_goal = path.poses[idx]
                #rospy.loginfo(f"\nRobot Pos: ({self.ttbot_pose.pose.position.x},{self.ttbot_pose.pose.position.y},{self.ttbot_pose.pose.position.z})\nCurrent goal: ({current_goal.pose.position.x}, {current_goal.pose.position.y},{current_goal.pose.position.z})")
                speed,heading, pitch = self.path_follower(self.ttbot_pose,current_goal)
                # Provide a threshold that robot can be within to reach goal position
                THRESHOLD = 0.08
                """if math.fabs(self.ttbot_pose.pose.position.x - self.goal_pose.pose.position.x) <= THRESHOLD and \
                   math.fabs(self.ttbot_pose.pose.position.y - self.goal_pose.pose.position.y) <= THRESHOLD and \
                   math.fabs(self.ttbot_pose.pose.position.z - self.goal_pose.pose.position.z) <= THRESHOLD:
                    print("Complete")
                    self.move_ttbot(0,0,0)                          # Set robot movement to 0
                else:"""
                self.move_ttbot(speed,heading,pitch)                # Set robot movement using speed and heading
            self.rate.sleep()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))       

if __name__ == "__main__":
    rospy.init_node("glider_navigator", anonymous=True)
    glider_nav = GliderNavigator()
    glider_nav.init_app()
    try:
        glider_nav.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Glider navigation has been terminated.")