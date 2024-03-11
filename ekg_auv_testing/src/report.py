import rospkg
import numpy as np
import math
import csv
import os
import matplotlib.pyplot as plt

from Graphing3D import PerformanceMetrics, Graphing3D

title = "Helical Glide Test"
  
time_flag = True
g3d = Graphing3D(time_flag)

#labels = ["true_pose", "dead_reckoning", "avg", "weighted_avg", "closest_neighbor"]
labels = ["true_pose", "dead_reckoning", "weighted_avg", "weighted_avg_with_dead_reckoning"]
timestamp = "2024-03-09_23:26:18"

pm = PerformanceMetrics()

g3d.get_data_from_csv(labels, timestamp)
data_pos = []
data_x  = []
data_y = []
data_z = [] 
ground_truth = g3d.paths[labels[0]].path
for i in range(1,len(labels)):
    method = g3d.paths[labels[i]].path
    metrics = pm.get_pose_diff_metrics(ground_truth, method, labels[i])
    data_pos.append(metrics)
    metrics = pm.get_single_val_metrics(ground_truth, method, "x")
    data_x.append(metrics)
    metrics = pm.get_single_val_metrics(ground_truth, method, "y")
    data_y.append(metrics)
    metrics = pm.get_single_val_metrics(ground_truth, method, "z")
    data_z.append(metrics)

collection = [data_pos, data_x, data_y, data_z]
order = ["Position", "X", "Y", "Z"]

cols = ('RSME', 'Mean', 'Standard Deviation')
g3d.show_table(f"Relative Pose Estimation", timestamp, labels[1:], cols, collection, order)
g3d.show_plot(title, timestamp)