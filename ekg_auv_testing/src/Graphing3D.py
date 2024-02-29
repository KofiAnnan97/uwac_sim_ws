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
    def __get_distance(self,curr_pose, next_pose):
        sum_of_squares = math.pow(curr_pose.position.x - next_pose.position.x, 2) + \
                         math.pow(curr_pose.position.y - next_pose.position.y, 2) + \
                         math.pow(curr_pose.position.z - next_pose.position.z, 2)
        dist = math.sqrt(math.fabs(sum_of_squares))
        return dist
    def calculate_mse(self, estimate, true):
        pass

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

    def save_plot(self, title, dir_name):
        from datetime import datetime
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("ekg_auv_testing")
        graphs_path = os.path.join(pkg_path, "graphs")
        dir_path = os.path.join(graphs_path, dir_name)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        timestamp = datetime.now().isoformat('_', timespec='seconds')
        
        new_title = re.sub(r'\s+', '_', title)
        plt.savefig(os.path.join(dir_path, f'{timestamp}_{new_title}.png'))

    def show_plot(self, title_name):
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
        self.save_plot(title_name, "visual")

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
            self.save_plot(title_name, "performance")
        except:
            print("Time data not given.")

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
        self.show_plot(title)

    def send_data_to_csv(self, label, timestamp, x, y, z, time):
        logs_path = self.get_log_path()
        path_name = re.sub(r'\s+', '_', label).lower()
        file_name = f'{timestamp}_{path_name}.csv'
        path_path = os.path.join(logs_path, path_name)
        file_path = os.path.join(path_path, file_name)

        with open(file_path, 'a', newline='') as cw:
            writer = csv.writer(cw)
            writer.writerow([time, x, y, z])

###########
# TESTING #
###########
"""if __name__ == "__main__":
    try:
        ########
        # Data #
        ########
        time_flag = True
        g3d = Graphing3D(time_flag)

        theta = np.linspace(-3*np.pi, 3*np.pi, 100)
        z =np.linspace(-9, 0, 100)
        r = z**2 +1
        x = r*np.sin(theta)
        y = r*np.cos(theta)

        theta2 = np.linspace(-5*np.pi, 5*np.pi, 100)
        z2 =np.linspace(-6, 0, 100)
        r2 = z2**2 +1
        x2 = r2*np.sin(theta)
        y2 = r2*np.cos(theta)

        theta3 = np.linspace(-7*np.pi, 7*np.pi, 100)
        z3 =np.linspace(-4, 0, 100)
        r3 = z3**2 +1
        x3 = r3*np.sin(theta)
        y3 = r3*np.cos(theta)

        theta4 = np.linspace(-9*np.pi, 9*np.pi, 100)
        z4 =np.linspace(-11, 0, 100)
        r4 = z4**2 +1
        x4 = r4*np.sin(theta)
        y4 = r4*np.cos(theta)

        #################################
        # General Functionality Testing #
        #################################
        #time = 0
        #path_str = "test_1"
        #g3d.add_path(path_str, "Test Label 1")
        #for path_name in g3d.paths:
        #    for i in range(len(x)):
        #        g3d.add_path_point(path_str, x[i], y[i], z[i], time)
        #        time += 1 

        #time = 0
        #path_str = "test_2"
        #g3d.add_path(path_str, "Test Label 2")
        #for path_name in g3d.paths:
        #    for i in range(len(x2)):
        #        g3d.add_path_point(path_str, x2[i], y2[i], z2[i], time)
        #        time += 1

        #time = 0
        #path_str = "test_3"
        #g3d.add_path(path_str, "Test Label 3")
        #for path_name in g3d.paths:
        #    for i in range(len(x3)):
        #        g3d.add_path_point(path_str, x3[i], y3[i], z3[i], time)
        #        time += 1

        #time = 0
        #path_str = "test_4"
        #g3d.add_path(path_str, "Test Label 4")
        #for path_name in g3d.paths:
        #    for i in range(len(x4)):
        #        g3d.add_path_point(path_str, x4[i], y4[i], z4[i], time)
        #        time += 1

        #g3d.show_plot("Testing 3D Lines")
                
        ####################
        # CSV File Testing #
        ####################
        labels = ["Label 1", "Test 2", "Sample 3", "Example 4"]
        stamp = g3d.init_paths(labels)

        time = 0
        for i in range(len(x)):
            g3d.send_data_to_csv(labels[0], stamp, x[i], y[i], z[i], time)
            time += 1 

        time = 0
        for i in range(len(x2)):
            g3d.send_data_to_csv(labels[1], stamp, x2[i], y2[i], z2[i], time)
            time += 1

        time = 0
        for i in range(len(x3)):
            g3d.send_data_to_csv(labels[2], stamp, x3[i], y3[i], z3[i], time)
            time += 1

        time = 0
        for i in range(len(x4)):
            g3d.send_data_to_csv(labels[3], stamp, x4[i], y4[i], z4[i], time)
            time += 1

        g3d.graph_data_from_csv(labels, stamp, "Testing 3D Lines")

    except Exception as e:
        print(e)"""