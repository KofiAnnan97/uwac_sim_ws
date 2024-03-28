import rospkg
import os
import csv
import re
import math

from geometry_msgs.msg import Pose

data = dict()

def get_log_path():        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("ekg_auv_testing")
        logs_path = os.path.join(pkg_path, "logs")
        return logs_path

def get_data_from_csv(robot_name, timestamp):
        logs_path = get_log_path()
        robot_name = re.sub(r'\s+', '_', robot_name).lower()
        robot_path = os.path.join(logs_path, robot_name)
        file_name = f'{timestamp}_{robot_name}.csv'
        file_path = os.path.join(robot_path, file_name)

        with open(file_path, 'r') as cr:
            reader = csv.reader(cr)
            for row in reader:
                try:
                    stamp = int(row[0])
                    data[stamp] = row[1:]
                except:
                    pass

def retrieve_data_by_time(timestamp, b_stamp, e_stamp):
        tmp_dict = dict()
        path = f"/home/ekg_unix/GitHub/research/simulation/uuv_ws/src/ekg_auv_testing/logs/glider_1/{timestamp}_glider_1.csv"
        with open(path, 'r') as cr:
            reader = csv.reader(cr)
            for row in reversed(list(reader)):
                """if int(row[0]) <= b_stamp:
                    return pose_log
                elif int(row[0]) <= e_stamp: 
                    pose_log.insert(0, row)"""
                try:
                    stamp = int(row[0])
                    if stamp >= b_stamp and stamp <= e_stamp:
                        tmp_dict[stamp] = row[1:]
                    elif stamp < b_stamp:
                         break
                except Exception as e:
                    print(e)
        #print(tmp_dict)
        return dict(sorted(tmp_dict.items()))

def get_new_pose_from_logs(data, starting_pose):
    final_pose = Pose()
    final_pose.position.x = starting_pose.position.x
    final_pose.position.y = starting_pose.position.y
    final_pose.position.z = starting_pose.position.z

    final_pose.orientation.x = starting_pose.orientation.x
    final_pose.orientation.y = starting_pose.orientation.y
    final_pose.orientation.z = starting_pose.orientation.z
    final_pose.orientation.w = starting_pose.orientation.w 

    prev_depth = 0
    prev_time = 0
    aoa_deg = -4

    for key, val in data.items():
        print(val)
        rpy = val[9:]
        if float(rpy[1]) < 0:
            glide_angle = float(rpy[1]) + aoa_deg
        else:
            glide_angle = float(rpy[1]) + aoa_deg
        
        depth = float(val[2])
        #print(depth)
        dz = depth - prev_depth
        #print(dz) 
        prev_depth = depth

        time = int(key)
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
        final_pose.position.x += d_x
        final_pose.position.y += d_y
        final_pose.position.z += dz 
    final_pose.position.z -= starting_pose.position.z
    return final_pose

if __name__ == "__main__":
    robot_name = "glider_1"
    timestamp = "2024-03-18_10:50:55"
    #get_data_from_csv(robot_name, timestamp)

    start_stamp = 7
    end_stamp = 15
    data = retrieve_data_by_time(timestamp, start_stamp, end_stamp)
    
    #print(data)
    starting_pose = Pose()
    starting_pose.position.x = 0 #float(data[start_stamp][0])
    starting_pose.position.y = 0 #float(data[start_stamp][1])
    starting_pose.position.z = -29.99 #float(data[start_stamp][2])
    final_pose = get_new_pose_from_logs(data,starting_pose)
    print(final_pose.position)
    