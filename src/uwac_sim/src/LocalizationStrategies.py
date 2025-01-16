#!/usr/bin/env python3

import rospy
import math
import json

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from rospy_message_converter import message_converter, json_message_converter

from uwac_sim.msg import VehiclePose, bcn_frame_array, bcn_pose_array
from Logger import GliderLogger

class Strats:
    def __init__(self):
        self.start_time = 0
        self.prev_start_time = -1
        self.rate = rospy.Rate(10)
        
        self.pose_estimates = dict()
        self.neighbors = dict()
        """self.neighbors ={"BEACON_ID0": None,
                         "BEACON_ID1": None,
                         "BEACON_ID2": None,
                         "BEACON_ID3": None,
                         "BEACON_ID4": None,
                         "BEACON_ID5": None,
                         "BEACON_ID6": None,
                         "BEACON_ID7": None,
                         "BEACON_ID8": None,
                         "BEACON_ID9": None,
                         "BEACON_ID10": None,
                         "BEACON_ID11": None,
                         "BEACON_ID12": None,
                         "BEACON_ID13": None,
                         "BEACON_ID14": None,
                         "BEACON_ID15": None}"""

        #print(rospy.get_param_names())
        self.glider_name = "glider_1" #rospy.get_param("~namespace")
     
        POSE_TOPIC = "/%s/pose_estimates"%self.glider_name #rospy.get_param("~pose_est_topic")
        NEIGHBOR_TOPIC = "/%s/neighbors"%self.glider_name #rospy.get_param("~neighbors_topic")
        #GLIDER_LOG_TOPIC = rospy.get_param("~glider_log_topic")
        LOC_STRAT_TOPIC = f"{self.glider_name}/strats/pose"

        rospy.Subscriber(POSE_TOPIC, bcn_pose_array, self.__poses_cbk)
        rospy.Subscriber(NEIGHBOR_TOPIC, bcn_frame_array, self.__neighbor_cbk)

        self.strat_pub = rospy.Publisher(LOC_STRAT_TOPIC, VehiclePose, queue_size=2)

    def __poses_cbk(self, msg):
        if msg is not None:
            for pose in msg.pose:
                self.pose_estimates[pose.bid] = pose

    def __neighbor_cbk(self, msg):
        if msg is not None:
            #print(f"Message: {msg}")
            for frame in msg.frame:
                neighbor = self.process_data(frame.data, "uwac_sim/bcn_pose")
                self.neighbors[neighbor.bid] = neighbor

    def __get_distance(self,curr_pose, next_pose):
        sum_of_squares = math.pow(curr_pose.x - next_pose.x, 2) + \
                         math.pow(curr_pose.y - next_pose.y, 2) + \
                         math.pow(curr_pose.z - next_pose.z, 2)
        dist = math.sqrt(math.fabs(sum_of_squares))
        return dist

    def process_data(self, msg_str, type):
        try:
            #print(msg_str)
            #msg_obj = json.loads(msg_str)
            rosmsg = json_message_converter.convert_json_to_ros_message(type, msg_str)
            return rosmsg
        except Exception as e:
            print(e)
            return None

    def get_simple_average(self):
        x, y, z, beacon_num = 0, 0, 0, 0
        for bid in self.pose_estimates.keys():
            try:
                node = self.pose_estimates[bid]
                x += node.x
                y += node.y
                z += node.z
                beacon_num += 1
            except:
                pass
        if beacon_num > 0:
            tmp_pose = VehiclePose()
            tmp_pose.vehicle_name = "avg"
            tmp_pose.x = x/beacon_num
            tmp_pose.y = y/beacon_num
            tmp_pose.z = z/beacon_num
            self.strat_pub.publish(tmp_pose)
            return tmp_pose
        return None

    def get_weighted_average(self, time_interval):
        x, y, z, weight_num = 0, 0, 0, 0
        curr_time = rospy.Time.now()

        avg_stamp = 0
        closest_stamp = rospy.Time(0, 0)
        furthest_stamp = rospy.Time(200000, 0)
        for bid in self.pose_estimates.keys():
            try:
                node = self.pose_estimates[bid]
                if closest_stamp.secs < node.stamp.secs:
                    closest_stamp = node.stamp
                if furthest_stamp.secs > node.stamp.secs:
                    furthest_stamp = node.stamp
                denom = math.pow((curr_time.secs - node.stamp.secs), 2)
                weight = round(9*(time_interval/denom), 2)
                avg_stamp += float(f"{node.stamp.secs}.{node.stamp.nsecs}") * weight
                x += node.x * weight
                y += node.y * weight
                z += node.z * weight
                weight_num += weight
            except Exception as e:
                print(f"Weighted Avg Func: {e}")
        #rospy.loginfo(f"----------------------------------------")
        if weight_num > 0:
            tmp_pose = VehiclePose()
            tmp_pose.vehicle_name = "weighted_avg"
            #tmp_pose.header.stamp = closest_stamp
            tmp_pose.stamp = rospy.Time.from_sec(avg_stamp/weight_num)
            #tmp_pose.header.stamp = furthest_stamp
            tmp_pose.x = x/weight_num
            tmp_pose.y = y/weight_num
            tmp_pose.z = z/weight_num
            self.strat_pub.publish(tmp_pose)
            return tmp_pose
        return None


    # ISSUE: Pose estimates are not being populated 

    def get_closest_neighbor(self):
        try:
            closest_dist = float("inf")
            beacon_id = -1
            for bid in self.pose_estimates.keys():
                e_pos = self.pose_estimates[bid]
                b_pos = self.neighbors[bid]
                dist = self.__get_distance(b_pos, e_pos)
                if dist < closest_dist:
                    closest_dist = dist
                    beacon_id = bid

            tmp_pose = VehiclePose()
            tmp_pose.vehicle_name = "closest_neighbor"
            tmp_pose.stamp = self.pose_estimates[beacon_id].stamp
            tmp_pose.x = self.pose_estimates[beacon_id].x
            tmp_pose.y = self.pose_estimates[beacon_id].y
            tmp_pose.z = self.pose_estimates[beacon_id].z
            self.strat_pub.publish(tmp_pose)

            return self.pose_estimates[beacon_id]
        except Exception as e:
            #rospy.loginfo("Neighbors could not be found.")
            #print(f"Get Closest Neighbor: {e}")
            return None

    def get_weighted_average_deadreckoning(self, wavg_pose, g_logger):
        if wavg_pose is not None:
            try:
                start_time = wavg_pose.stamp.secs
                if start_time == self.prev_start_time:
                    start_time = self.start_time
                elif start_time == 0:
                    start_time = self.start_time
                start_time += 1
                end_time = rospy.Time.now().secs

                data = g_logger.retrieve_data_by_time(start_time, end_time)
                fuse_pose = self.get_new_pose_from_logs(data, wavg_pose)

                #print(f"\nTime span: {start_time} to {end_time}")
                #print(f"Start interval: {self.start_time}")
                #print(f"WAVG Time: {wavg_pose.header.stamp.secs}")
                #print(f"New Time: {fuse_pose.header.stamp.secs}")

                self.prev_start_time = start_time
                self.start_time = end_time
                return fuse_pose
            except Exception as e:
                print(f"Error in wadr func.\n{e}")     
        return None

    def get_new_pose_from_logs(self, data, starting_pose):
        final_pose = VehiclePose()
        final_pose.x = starting_pose.x
        final_pose.y = starting_pose.y
        final_pose.z = starting_pose.z
        final_pose.roll = starting_pose.roll
        final_pose.pitch = starting_pose.pitch
        final_pose.yaw = starting_pose.yaw

        prev_depth = 0
        prev_time = 0
        aoa_deg = -24 #-27 #-2.7

        for val in data:
            rpy = val[10:]
            if float(rpy[1]) < 0:
                glide_angle = float(rpy[1]) + aoa_deg
            else:
                glide_angle = float(rpy[1]) + aoa_deg
            
            depth = float(val[3])
            dz = depth - prev_depth 
            prev_depth = depth

            time = int(val[0])
            dt = time - prev_time
            prev_time = time

            if abs(float(rpy[1])) < math.pi/100.0:
                v_x = 0.0
                v_y = 0.0
            else:
                #yaw_deg = float(rpy[2])*(180/math.pi)
                v_x = dz/(dt*math.tan(glide_angle))*math.sin(math.pi/2.0-float(rpy[2]))
                v_y = dz/(dt*math.tan(glide_angle))*math.cos(math.pi/2.0-float(rpy[2]))
            d_x = v_x * dt
            d_y = v_y * dt
            final_pose.stamp.secs = int(val[0])
            final_pose.x -= d_x
            final_pose.y -= d_y
        if(len(data) > 0):
            final_pose.z = depth
        return final_pose
    
    def run(self):
        while not rospy.is_shutdown():
            avg = self.get_simple_average()
            wavg = self.get_weighted_average(10)
            cn = self.get_closest_neighbor()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("loc_strats", anonymous=True)
    ls = Strats()
    try:
        ls.run()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Localization Strategy node has terminated.")