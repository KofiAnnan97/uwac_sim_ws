import numpy as np
import argparse

from Visualizer import PerformanceMetrics, Graphing3D

#labels = ["true_pose", "avg", "weighted_avg", "closest_neighbor"]
#labels = ["true_pose", "dead_reckoning", "avg", "weighted_avg", "closest_neighbor"]
labels = ["true_pose", "dead_reckoning", "weighted_avg", "weighted_avg_with_dead_reckoning"]
#labels = ["true_pose","wavg_15","wavg_10","wavg_5","wavg_30"]

ls = "Localization Strats"
wt = "WADR Testing"

default_filename = wt
#timestamp = "2024-03-14_23:29:08"


def main():
    parser = argparse.ArgumentParser(
                        prog='ReportGenerator',
                        description='A tool to assist in generating graphs and tables for the data collected using this ROS Package.')
    parser.add_argument('-f', '--filename', default=default_filename, action='store', help='Provide a title for file generation.')
    parser.add_argument('-t', '--timestamp', action='store', help="Provide a timestamp of the following format: YYYY-MM-DD_hh:mm:ss")

    args = parser.parse_args()
    title = args.filename
    timestamp = args.timestamp
    
    time_flag = True

    pm = PerformanceMetrics()

    g3d = Graphing3D(time_flag)
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
    g3d.show_table(f"{title} ATE", timestamp, labels[1:], cols, collection, order)
    g3d.show_plot(title, timestamp)
    print("Complete")


if __name__ == "__main__":
    main()