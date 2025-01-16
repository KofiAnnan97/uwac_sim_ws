# Useful Scripts
1. Graphing3D.py
2. Logger.py
3. report.py

# Graphing3D.py

## Implementing graphing via system memory

Initialization
````python
grapher = Graphing3D()
````

Adding a path.
````python
grapher.add_path("AUV_TRUE_POSE_1", "Ground Truth")
````

Add point to path.
````python
grapher.add_path_point("AUV_TRUE_POSE_1", x, y, z, rospy.Time(0,0).secs)
````

## Implmenting graphing via csv files

Initialization
````python
labels = ["true_pose", "dead_reckoning", "avg", "weighted_avg", "closest_neighbor"]
grapher = Graphing3D(include_time=True)
log_stamp = grapher.init_paths(labels)
````

Add point to csv file
````python
grapher.send_data_to_csv("AUV_TRUE_POSE_1", log_stamp, x, y, z, rospy.Time(0,0).secs)
````

# Logger.py

# report.py
