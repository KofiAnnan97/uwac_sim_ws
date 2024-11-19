# Pre-requirements
1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. [Dave](https://field-robotics-lab.github.io/dave.doc/contents/installation/Clone-Dave-Repositories/)
3. Gazebo 11
4. [glider_hybrid_whoi](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi)
5. [FRL Msgs](https://github.com/Field-Robotics-Lab/frl_msgs)
5. Nps_uw_sensors_gazebo
6. seatrac_usbl (Githuib repo from naslab-projects)

# Python libraries
1. rospy_message_converter

Add this package to the ~/uuv_ws/src 
use catkin_make



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
