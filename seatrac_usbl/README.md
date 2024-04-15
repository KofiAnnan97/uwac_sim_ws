# seatrac_usbl
ROS package to interface with SeaTrac Beacons x110/x150 with ROS framework.

## Hardware Description and Links
  The seatrac_usbl deals with the code to interface with the `Blueprint Subsea Seatrac X150` (https://www.blueprintsubsea.com/pages/product.php?PN=BP00795) and 
  `lueprint Subsea Seatrac X110` (https://www.blueprintsubsea.com/pages/product.php?PN=BP00843) beacons. Both beacons allow two way accoustic communicaiton and
  enable the ranging/localization of the x110. This code requires at least one of the beacons and:
  1. Serial to USB concerter
  2. Wet Mate connector to attach to the 5-pin data/power connector
  3. Raspberry Pi
  4. 12 - 24 volt power supply
  
## Ros Messages (./msg))

### IverOSI.msg
Associated Topics: 

Data:
1. -----


### bcn_array.msg
Associated Topics: 

Data:
1. -----

### bcn_frame_array.msg
Associated Topics: 

Data:
1. -----

### bcn_pose.msg
Associated Topics: 

Data:
1. -----

### bcn_pose_array.msg
Associated Topics: 

Data:
1. -----

### bcn_status.msg
Associated Topics: 

Data:
1. -----


### bcn_status_array.msg
Associated Topics: 

Data:
1. -----

### loc.msg
Associated Topics: `/gps`

Data:
1. lat: Current latitude of the boat - [float32 -> degrees]
2. long: Current longitude of boat - [float32 -> degrees]

Description: Message that contains GPS location information taken from the GPS module.


### head.msg
Associated Topics: `/heading`

Data
1. heading: The current compass bearing of the vehicle - [float32 -> Degrees]
2. xacc: The current acceleration of the vehicle along its primary axis (surge) - [float32 -> m/s^2]
3. yacc: The current acceleration of the vehicle along its seconday axis (sway) - [float32 -> m/s^2]
4. zacc: The current acceleration of the vehicle along its primary axis - [float32 -> m/s^2]
5. roll: The vehicles roll (about x-axis) value - [float32 -> Degrees]
6. pitch: The vehicles pitch (about y/sway axis) value - [float32 -> Degrees]

Description: Message that contains all information regarding the vehicles current rotational poses (x,y,z from loc.msg)


## Scripts

### config_bs.sh

### config_fs.sh

### env_bs.sh

### env_fs.sh

### replay_aut_cat.sh

### start_aut_cat.sh
This script launches the neccessary ROS nodes in the frontseat's aut_cat package, the backseats pm_plan package, and the backseats Seatrac_pkg package. 
This allows the vehicle actuator management software, the vehicles path planning software, and the seatrack software to all be launched under a single ROS master node,
thus allowing all the packages to communicate with each other across both computers. `If you want to launch the Autonomous Surface Vehicle on a deployment this is 
the script that you must run`. If the beacons are not attached to the vehcile then the associated beacon nodes will succesfulyl exit without causing issues.

### start_rocore.sh

### sync_pc_pi

## Nodes (./src)
### AcousticAnalysis.py
### BasicTypes.py
### Beacon.py
### CommHandler.py
### CsvLogger.py
### CustCommTest.py
### GpsCalculations.py
### GpsTracker.py
### MessageTypes.py

## Dependencies





