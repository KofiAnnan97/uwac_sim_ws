# Underwater Acoustic Communication Workspace
A ROS Neotic workspace designed to emulate and test acoustic communication in underwater environments.

## Table of Contents
- [Installation](#installation)
    - [Pre-Requistes](#pre-requistes)
    - [ROS packages](#ros-packages)
    - [Python Libraries](#python-libraries)
- [Execution](#execution)
- [Structure](#structure)

## Installation

### Pre-requistes
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Gazebo 11
- Python 3.8 or newer

### ROS packages
- [Dave](https://field-robotics-lab.github.io/dave.doc/contents/installation/Clone-Dave-Repositories/)
    
    Installing Dave and supported ROS packages:
    ```bash
    mv uwac_sim_ws/ uuv_ws
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
- [Glider_hybrid_whoi](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi)
- [FRL_Vehicle_Msgs](https://github.com/Field-Robotics-Lab/frl_msgs)
- [Gazebo ros packages](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
- [Nps_uw_sensors_gazebo](https://github.com/Field-Robotics-Lab/nps_uw_sensors_gazebo)
- [seatrac_pkg](https://github.com/jalilChavez/seatrac_usbl) (optional private Github repo)

### Python Libraries
- Rospy-message-converter 
- PyYaml
- Matplotlib
- Numpy

Install via pip:
```bash
pip install rospy-message-converter pyyaml matplotlib numpy
```

If you are this running in Visual Studio Code make sure to install the ```C/C++ Extension Pack``` by Microsoft.

## Execution
The following actions can be run:
- Only running the static beacons 
    ```bash 
    roslaunch uwac_sim beacons_test.launch
    ```
- Only running the glider
    ```bash 
    roslaunch uwac_sim glider_test.launch
    ```
- Running the glider and beacons together
    ```bash 
    roslaunch uwac_sim static_beacon_localization.launch
    ```

## Structure
### Acoustic Communication
![Communication Framework](/src/uwac_sim/graphics/framework.png)
Transceiver Topics
- /USBL/transceiver_<transceiver_id>/transponder_location  => Stores location of transponder.
- /USBL/transceiver_<transceiver_id>/mode                  => Changes the interrogation mode between the common and individual channels. 
- /USBL/transceiver_<transceiver_id>/channel_switch        => Changes the transponder id.
- /USBL/transceiver_<transceiver_id>/command_request       => Command used to send a request to a specific transciever.

Transponder Topics
- /USBL/transponder_<transponder_id>/command_response      => Command used to send a response to a specific transponder.
- /USBL/transponder_<transponder_id>/channel_switch        => Changes the transceiver id.        
- /USBL/transponder_<transponder_id>/ping                  => Send a message to a specific transponder based on id.

Common Topics
- /USBL/common_ping                                        => A common channel where the transponder and transceiver can send or recieve messages.