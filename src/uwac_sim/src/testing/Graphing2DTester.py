import numpy as np
import importlib.machinery
import importlib.util
import rospkg
import os

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("uwac_sim")
src_path = os.path.join(pkg_path, "src")
graphing_path = os.path.join(src_path, 'Visualizer.py')

loader = importlib.machinery.SourceFileLoader('Graphing2D', graphing_path)
spec = importlib.util.spec_from_loader('Graphing2D', loader)
Visualizer = importlib.util.module_from_spec(spec)
print(Visualizer)
loader.exec_module(Visualizer)

AUV_TRUE_POSE_1 = "true_pose"
AUV_EST_POSE_1 = "dead_reckoning"
AUV_BEACON_POSE_1 = "avg"
AUV_BEACON_POSE_2 = "weighted_avg"
AUV_BEACON_POSE_3 = "closest_neighbor"
AUV_FUSION_POSE_1 = "avg_with_dead_reckoning"
AUV_FUSION_POSE_2 = "weighted_avg_with_dead_reckoning"
AUV_FUSION_POSE_3 = "closest_neighbor_with_dead_reckoning"
labels = [AUV_TRUE_POSE_1, AUV_EST_POSE_1, AUV_BEACON_POSE_1, AUV_BEACON_POSE_2, AUV_BEACON_POSE_3]
    
    
##################
# From CSV Files #
##################

"""time_flag = True
g3d = Graphing3D.Graphing3D(time_flag)
timestamp = "2024-03-28_17:08:14"
g3d.graph_data_from_csv(labels, timestamp, "Helical Glide Test")"""

try:
    ########
    # Data #
    ########
    time_flag = True
    viz = Visualizer.Graphing2D(time_flag)

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

    labels = ["Label 1", "Test 2", "Sample 3", "Example 4"]
    stamp = viz.init_paths(labels)

    time = 0
    for i in range(len(x)):
        viz.send_data_to_csv(labels[0], stamp, x[i], y[i], time)
        time += 1 

    time = 0
    for i in range(len(x2)):
        viz.send_data_to_csv(labels[1], stamp, x2[i], y2[i], time)
        time += 1

    time = 0
    for i in range(len(x3)):
        viz.send_data_to_csv(labels[2], stamp, x3[i], y3[i], time)
        time += 1

    time = 0
    for i in range(len(x4)):
        viz.send_data_to_csv(labels[3], stamp, x4[i], y4[i], time)
        time += 1

    viz.graph_data_from_csv(labels, stamp, "Testing 2D Lines")

except Exception as e:
    print(e)