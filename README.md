### Pre-Requistes
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Dave](https://field-robotics-lab.github.io/dave.doc/contents/installation/Clone-Dave-Repositories/)
    
    Installing Dave and support ROS packages:
    ```bash
    cd ~/uuv_ws/src
    git clone https://github.com/Field-Robotics-Lab/dave.git
    sudo pip3 install -U vcstool
    cd ~/uuv_ws/src
    vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
    ```
    If that does work try again use the following:
    ```bash
    cd ~/uuv_ws/src
    rm -rf dockwater ds_msgs ds_sim eca_a9 rexrov2 uuv_manipulators uuv_simulator
    vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
    ```     
- Gazebo 11
- [Glider_hybrid_whoi](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi)
- [FRL_Vehicle_Msgs](https://github.com/Field-Robotics-Lab/frl_msgs)
- [Gazebo ros packages](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
- Nps_uw_sensors_gazebo
- seatrac_pkg (Githuib repo from naslab-projects)

### Python Libraries
- Rospy-message-converter 
- PyYaml
- Matplotlib
- Numpy

Install via pip:
```bash
pip install rospy-message-converter pyyaml matplotlib numpy
```