import rospkg
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import csv
import os
import re

class XYZPath:
    def __init__(self):
        self.path = list()
        self.label = None

    def add_point(self, x, y, z):
        if x != None and y != None and z != None:
            pt = (x, y, z)
            self.path.append(pt)

    def set_label(self, label_str):
        self.label = label_str

    def get_label(self):
        return self.label

    def get_x_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[0], self.path))
        else:
            return None
    
    def get_y_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[1], self.path))
        else:
            return None
        
    def get_z_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[2], self.path))
        else:
            return None
        
    def to_str(self):
        if len(self.path) > 0:
            for pt in self.path:
                print(f"({pt[0]}, {pt[1]}, {pt[2]})")
        else:
            print("Empty")
        
class XYZTimePath:
    def __init__(self):
        self.path = list()
        self.label = None
    def add_point(self, x, y, z, time):
        if x != None and y != None and z != None and time != None:
            pt = (x, y, z, time)
            self.path.append(pt)

    def set_label(self, label_str):
        self.label = label_str

    def get_label(self):
        return self.label

    def get_x_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[0], self.path))
        else:
            return None
    
    def get_y_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[1], self.path))
        else:
            return None
        
    def get_z_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[2], self.path))
        else:
            return None
    
    def get_time_pts(self):
        if len(self.path) > 0:
            return list(map(lambda pt: pt[3], self.path))
        else:
            return None
        
    def to_str(self):
        if len(self.path) > 0:
            for pt in self.path:
                print(f"({pt[0]}, {pt[1]}, {pt[2]}, {pt[3]})")
        else:
            print("Empty")
        
class PerformanceMetrics:
    def __init__(self):
        pass

    # Calculate pose between two poses
    def calc_distance(self, ytrue, ymeas):
        sum_of_squares = math.pow(ymeas[0] - ytrue[0], 2) + \
                         math.pow(ymeas[1] - ytrue[1], 2) + \
                         math.pow(ymeas[2] - ytrue[2], 2)
        dist = math.sqrt(math.fabs(sum_of_squares))
        return dist
        
    # Calculate Accuracy
    def calc_acc(self, true_val, meas_val):
        try:
            return 100 - self.calc_err_rate(true_val, meas_val)
        except:
            return None
    
    # Calculate Error Rate
    def calc_err_rate(self, true_val, meas_val):
        try:
            return 100*(math.fabs(meas_val - true_val)/true_val)
        except:
            return None
        
    def get_index_by_time(self, timestamp, start_idx, poses):
        for i in range(start_idx, len(poses)):
            if poses[i][3] == timestamp:
                #print(f"Stamp: {timestamp}, Pose: {poses[i]}")
                return i
        return -1
    ###################################
    # Absolute Trajectory Error (ATE) #
    ################################### 

    # RPE Root Square Mean Error (Pose Difference)
    def calc_ate_rmse(self, ytrue, ymeas):
        try:
            sum = 0
            count = 0
            try:
                for i in range(len(ymeas)):
                    j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                    sum += math.pow(self.calc_distance(ytrue[j], ymeas[i]), 2)
                    count += 1
            except:
                pass
            sum /= count
            return round(math.sqrt(sum), 4)
        except:
            return None
        
    # RPE Mean (Pose Difference)
    def calc_ate_mean(self, ytrue, ymeas):
        mu = 0
        count = 0
        try:
            for i in range(len(ymeas)):
                j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                mu += self.calc_distance(ytrue[j], ymeas[i])
                count +=1
        except:
            pass
        mu /= count
        return round(mu, 4)

    # RPE Standard Deviation (Pose Difference)
    def calc_ate_sd(self, mu, ytrue, ymeas):
        std = 0
        count = 0
        try:
            for i in range(len(ymeas)):
                j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                dist = self.calc_distance(ytrue[j], ymeas[i])
                std += math.pow((dist - mu), 2)
                count += 1
        except:
            pass
        std /= count
        return round(math.sqrt(std), 4)
    
    def get_pose_diff_metrics(self, ytrue, ymeas, title):
        rmse = self.calc_ate_rmse(ytrue, ymeas)
        mu = self.calc_ate_mean(ytrue, ymeas)
        sd = self.calc_ate_sd(mu, ytrue, ymeas)
        return (rmse, mu, sd)
    
    # ATE Root Square Mean Error (Single Value)
    def calc_ate_rmse2(self, ytrue, ymeas, val_type):
        try:
            sum = 0
            count = 0
            try:
                for i in range(len(ymeas)):
                    j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                    if val_type == "x":
                        c_idx = 0
                    elif val_type == "y":
                        c_idx = 1
                    elif val_type == "z":
                        c_idx = 2
                    sum += math.pow(math.fabs(ytrue[j][c_idx] - ymeas[i][c_idx]), 2)
                    count += 1
            except:
                pass
            sum /= count
            return round(math.sqrt(sum), 2)
        except:
            return None
        
    # ATE Mean (Single Value)
    def calc_ate_mean2(self, ytrue, ymeas,val_type):
        mu = 0
        count = 0
        try:
            for i in range(len(ymeas)):
                j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                if val_type == "x":
                    c_idx = 0
                elif val_type == "y":
                    c_idx = 1
                elif val_type == "z":
                    c_idx = 2
                mu += math.fabs(ytrue[j][c_idx]- ymeas[i][c_idx])
                count +=1
        except:
            pass
        mu /= count
        return round(mu, 4)

    # ATE Standard Deviation (Single Value)    
    def calc_ate_sd2(self, mu, ytrue, ymeas, val_type):
        std = 0
        count = 0
        try:
            for i in range(len(ymeas)):
                j = self.get_index_by_time(ymeas[i][3], i, ytrue)
                if val_type == "x":
                    c_idx = 0
                elif val_type == "y":
                    c_idx = 1
                elif val_type == "z":
                    c_idx = 2
                std += math.pow((ytrue[j][c_idx] -ymeas[i][c_idx]) - mu, 2)
                count += 1
        except:
            pass
        std /= count
        return round(math.sqrt(std), 4)

    def get_single_val_metrics(self, ytrue, ymeas, val_type):
        rmse = self.calc_ate_rmse2(ytrue, ymeas, val_type)
        mu = self.calc_ate_mean2(ytrue, ymeas, val_type)
        sd = self.calc_ate_sd2(mu, ytrue, ymeas, val_type)
        return (rmse, mu, sd)
    
    #############################
    # Relative Pose Error (RPE) #
    #############################

    # Looking into how to implement this properly (https://arxiv.org/pdf/1910.04755.pdf)

class Graphing3D:
    def __init__(self, include_time = False):
        self.paths = dict()
        self.time_flag = include_time

    ################################
    # System Memory Implementation #
    ################################
    def add_path(self, key, label_str):
        if self.time_flag == False:
            if key not in self.paths.keys():
                self.paths[key] = XYZPath()
                self.paths[key].set_label(label_str)
            else:
                print(f"{key} is already in use.")
        elif self.time_flag == True:
            if key not in self.paths.keys():
                self.paths[key] = XYZTimePath()
                self.paths[key].set_label(label_str)
            else:
                print(f"{key} is already in use.")

    def add_path_point(self, path_name, x, y, z, time=None):
        if time is None:
            if path_name in self.paths.keys():
                self.paths[path_name].add_point(x, y, z)
            else:
                print(f"{path_name} does not exist in path dictionary.")
        else:
            if path_name in self.paths.keys():
                self.paths[path_name].add_point(x, y, z, time)

    def save_plot(self, title, dir_name, timestamp = None):
        from datetime import datetime
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("ekg_auv_testing")
        graphs_path = os.path.join(pkg_path, "graphs")
        dir_path = os.path.join(graphs_path, dir_name)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        if timestamp is None:
            timestamp = datetime.now().isoformat('_', timespec='seconds')
        new_title = re.sub(r'\s+', '_', title)
        plt.savefig(os.path.join(dir_path, f'{timestamp}_{new_title}.png'))

    def show_plot(self, title_name, stamp = None):
        print("Preparing graph visualization...")
        paths_copy = self.paths.copy()
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        plt.title(f"{title_name} Visual")
        #ax.set_autoscale_on = True

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Depth')

        for path_name in paths_copy:
            path = paths_copy[path_name]
            x = path.get_x_pts()
            y = path.get_y_pts()
            z = path.get_z_pts()
            #print(f"{path_name}\nX: {x}, \nY: {y}, \nZ: {z}\nLabel: {path.get_label()}\n")
            ax.plot(x, y, z, label=path.get_label())
        
        ax.set_autoscalex_on = True
        ax.set_autoscaley_on = True

        plt.legend()
        #plt.show()
        if stamp is None:
            self.save_plot(title_name, "visual")
        else:
            self.save_plot(title_name, "visual", stamp)

        try:
            fig, axs = plt.subplots(3, 1)
            fig.suptitle(f"{title_name} Performance")
            
            axs[0].set_ylabel('X')
            axs[1].set_ylabel('Y')
            axs[2].set_ylabel('Z')
            axs[2].set_xlabel('Time')

            for path_name in paths_copy:
                path = paths_copy[path_name]
                x = path.get_x_pts()
                y = path.get_y_pts()
                z = path.get_z_pts()
                t = path.get_time_pts()
                axs[0].plot(t, x, label=path.get_label())
                axs[1].plot(t, y, label=path.get_label())
                axs[2].plot(t, z, label=path.get_label())

            plt.legend()
            #plt.show()
            if stamp is None:
                self.save_plot(title_name, "performance")
            else:
                self.save_plot(title_name, "performance", stamp)
        except:
            print("Time data not given.")

    def show_table(self, title, timestamp, rows, cols, collection, order):
        print("Preparing performance metrics...")
        fig, axs = plt.subplots(len(collection),1)
        fig.patch.set_visible(False)

        for i in range(len(collection)):
            axs[i].axis('off')
            axs[i].axis('tight')
            table = axs[i].table(cellText=collection[i], colLabels=cols, rowLabels=rows, loc='center')
            table.scale(1, 1.5)
            axs[i].set_title(order[i], loc='left')

        plt.suptitle(title, size= 15)
        plt.figtext(0.95, 0.05, timestamp, horizontalalignment='right', size=10, weight='light')
        fig.tight_layout()
        self.save_plot(title, "metrics", timestamp)
        #plt.show()

    #############################
    # File-based Implementation #
    #############################
    def get_log_path(self):        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("ekg_auv_testing")
        logs_path = os.path.join(pkg_path, "logs")
        return logs_path

    def init_paths(self, labels):
        from datetime import datetime
        logs_path = self.get_log_path()
        timestamp = datetime.now().isoformat('_', timespec='seconds')

        for label in labels:
            import re
            new_label = re.sub(r'\s+', '_', label).lower()
            label_path = os.path.join(logs_path, new_label)
            if not os.path.exists(label_path):
                os.makedirs(label_path)
            file_name = f'{timestamp}_{new_label}.csv'
            file_path = os.path.join(label_path, file_name)

            with open(file_path, 'w', newline='') as cw:
                writer = csv.writer(cw)
                writer.writerow(["timestamp", "x", "y", "z"])
        return timestamp 

    def graph_data_from_csv(self, labels, timestamp, title):
        self.get_data_from_csv(labels, timestamp)
        self.show_plot(title, timestamp)

    def get_data_from_csv(self, labels, timestamp):
        logs_path = self.get_log_path()
        for label in labels:
            path_name = re.sub(r'\s+', '_', label).lower()
            path_path = os.path.join(logs_path, path_name)
            file_name = f'{timestamp}_{path_name}.csv'
            file_path = os.path.join(path_path, file_name)

            self.add_path(path_name, label)
            with open(file_path, 'r') as cr:
                reader = csv.reader(cr)
                for row in reader:
                    try:
                        self.add_path_point(path_name, round(float(row[1]), 2), round(float(row[2]), 2), round(float(row[3]), 2), float(row[0]))
                    except:
                        pass

    def send_data_to_csv(self, label, timestamp, x, y, z, time):
        logs_path = self.get_log_path()
        path_name = re.sub(r'\s+', '_', label).lower()
        file_name = f'{timestamp}_{path_name}.csv'
        path_path = os.path.join(logs_path, path_name)
        file_path = os.path.join(path_path, file_name)

        with open(file_path, 'a', newline='') as cw:
            writer = csv.writer(cw)
            writer.writerow([time, x, y, z])