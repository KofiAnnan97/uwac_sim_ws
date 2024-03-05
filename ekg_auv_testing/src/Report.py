import rospkg
import numpy as np
import math
import csv
import os
import matplotlib.pyplot as plt

from Graphing3D import PerformanceMetrics, Graphing3D

title = "Relative Pose Estimation"

time_flag = True
g3d = Graphing3D(time_flag)

labels = ["true_pose", "dead_reckoing", "avg", "weighted_avg", "closest_neighbor"]
timestamp = "2024-03-04_00:42:54"

pm = PerformanceMetrics()

g3d.get_data_from_csv(labels, timestamp)
data = []
ground_truth = g3d.paths[labels[0]].path
for i in range(1,len(labels)):
    method = g3d.paths[labels[i]].path
    metrics = pm.print_metrics(ground_truth, method, labels[i])
    data.append(metrics)

cols = ('RSME', 'Mean', 'Standard Deviation')

g3d.show_table(title, timestamp, labels[1:], cols, data)