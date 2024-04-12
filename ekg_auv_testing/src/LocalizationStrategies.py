import rospy
import math

from ekg_auv_testing.msg import VehiclePose
from Logger import GliderLogger

class Strats:
    def __init__(self, glider_name, stamp, glider_topic):
        self.start_time = 0
        self.prev_start_time = -1
        self.g_logger = GliderLogger(glider_name, stamp, glider_topic)
        self.g_logger.set_glider_log(glider_name, stamp)

    def __get_distance(self,curr_pose, next_pose):
        sum_of_squares = math.pow(curr_pose.position.x - next_pose.position.x, 2) + \
                         math.pow(curr_pose.position.y - next_pose.position.y, 2) + \
                         math.pow(curr_pose.position.z - next_pose.position.z, 2)
        dist = math.sqrt(math.fabs(sum_of_squares))
        return dist

    def get_simple_average(self, pose_estimates):
        x, y, z, beacon_num = 0, 0, 0, 0
        for bid in pose_estimates.keys():
            try:
                node = pose_estimates[bid]
                # rospy.loginfo(f"Beacon {bid}: ({node.pose.position.x}, {node.pose.position.y}, {node.pose.position.z})")
                x += node.pose.position.x
                y += node.pose.position.y
                z += node.pose.position.z
                beacon_num += 1
            except:
                pass
        if beacon_num > 0:
            tmp_pose = VehiclePose()
            tmp_pose.pose.position.x = x/beacon_num
            tmp_pose.pose.position.y = y/beacon_num
            tmp_pose.pose.position.z = z/beacon_num
            return tmp_pose
        return None

    def get_weighted_average(self, time_interval, pose_estimates):
        x, y, z, weight_num = 0, 0, 0, 0
        curr_time = rospy.Time.now()

        avg_stamp = 0
        closest_stamp = rospy.Time(0, 0)
        furthest_stamp = rospy.Time(200000, 0)
        #rospy.loginfo(f"Current Time: {curr_time.secs}")
        for bid in pose_estimates.keys():
            try:
                node = pose_estimates[bid]
                if closest_stamp.secs < node.header.stamp.secs:
                    closest_stamp = node.header.stamp
                if furthest_stamp.secs > node.header.stamp.secs:
                    furthest_stamp = node.header.stamp
                # time_interval = 15
                # Reward estimations based on transmission time in seconds
                denom = math.pow((curr_time.secs - node.header.stamp.secs), 2)
                weight = round(9*(time_interval/denom), 2)
                #rospy.loginfo(f"beacon_{bid}|| Time: {node.header.stamp.secs}, Weight: {weight}")
                #rospy.loginfo(f"Time Delay: {curr_time - node.header.stamp.to_sec()}, Weight: {weight}")
                #rospy.loginfo(f"Beacon {bid}: ({node.pose.position.x}, {node.pose.position.y}, {node.pose.position.z})")
                avg_stamp += float(f"{node.header.stamp.secs}.{node.header.stamp.nsecs}") * weight
                x += node.pose.position.x * weight
                y += node.pose.position.y * weight
                z += node.pose.position.z * weight
                weight_num += weight
            except Exception as e:
                print(f"Weighted Avg Func: {e}")
        #rospy.loginfo(f"----------------------------------------")
        if weight_num > 0:
            tmp_pose = VehiclePose()
            tmp_pose.header.stamp = closest_stamp
            # tmp_pose.header.stamp = rospy.Time.from_sec(avg_stamp/weight_num)
            # tmp_pose.header.stamp = furthest_stamp
            tmp_pose.pose.position.x = x/weight_num
            tmp_pose.pose.position.y = y/weight_num
            tmp_pose.pose.position.z = z/weight_num
            return tmp_pose
        return None

    def get_closest_neighbor(self, pose_estimates, neighbors):
        try:
            closest_dist = float("inf")
            beacon_id = -1
            #print(f"Neighbors: {self.neighbors}")
            # print(f"Estimates: {self.beacon_estimates.keys()}")
            for bid in pose_estimates.keys():
                e_pos = pose_estimates[bid]
                b_pos = neighbors[bid]
                #rospy.loginfo(f"Beacon {bid}: ({e_pos.pose.position.x}, {e_pos.pose.position.y}, {e_pos.pose.position.z})")

                dist = self.__get_distance(b_pos.pose, e_pos.pose)
                if dist < closest_dist:
                    closest_dist = dist
                    beacon_id = bid
            return pose_estimates[beacon_id]
        except Exception as e:
            rospy.loginfo("Neighbors could not be found.")
            print(e)
            return None

    def get_weighted_average_deadreckoning(self, wavg_pose):
        if wavg_pose is not None:
            try:
                start_time = wavg_pose.header.stamp.secs
                if start_time == self.prev_start_time:
                    start_time = self.start_time
                elif start_time == 0:
                    start_time = self.start_time
                start_time += 1
                end_time = rospy.Time.now().secs

                data = self.g_logger.retrieve_data_by_time(start_time, end_time)
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
                #pass     
        return None

    def get_new_pose_from_logs(self, data, starting_pose):
        final_pose = VehiclePose()
        final_pose.pose.position.x = starting_pose.pose.position.x
        final_pose.pose.position.y = starting_pose.pose.position.y
        final_pose.pose.position.z = starting_pose.pose.position.z

        final_pose.pose.orientation.x = starting_pose.pose.orientation.x
        final_pose.pose.orientation.y = starting_pose.pose.orientation.y
        final_pose.pose.orientation.z = starting_pose.pose.orientation.z
        final_pose.pose.orientation.w = starting_pose.pose.orientation.w 

        prev_depth = 0
        prev_time = 0
        aoa_deg = -2.7 #-2.8

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
            final_pose.header.stamp.secs = int(val[0])
            final_pose.pose.position.x += d_x
            final_pose.pose.position.y += d_y
        if(len(data) > 0):
            final_pose.pose.position.z = depth #data[-1:][3]
        #rospy.loginfo(f"\n Positon:\n{starting_pose.pose.position}\n New Position:\n{final_pose.pose.position}")
        return final_pose