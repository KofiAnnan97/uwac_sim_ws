
#!/bin/bash

# ===========================================================================
# File        : sync_pc_pi.bat
# Author      : Jalil Chavez
# Descirption : Synchronizes the content of repository in local folder and
#               the content of the remote folder. REMOTE_FOLDER and
#               LOCAL_FOLDER have to be reassigned in case the script is used
#               in a different setup.
# ===========================================================================

REMOTE_FOLDER_1=/home/pi/catkin_ws/src/seatrac_pkg/
REMOTE_USER_1=pi
REMOTE_IP_1=192.168.15.20

REMOTE_FOLDER_2=/home/pi/catkin_ws/src/seatrac_pkg/
REMOTE_USER_2=pi
REMOTE_IP_2=192.168.15.21

LOCAL_FOLDER=../

echo ===============================
echo Synchronizing Folders
echo ===============================
echo Local Path : $LOCAL_FOLDER
echo ===============================
echo Remote Path: $REMOTE_FOLDER_1
echo Remote User: $REMOTE_USER_1
echo Remote IP  : $REMOTE_IP_1
echo ===============================
echo Remote Path: $REMOTE_FOLDER_2
echo Remote User: $REMOTE_USER_2
echo Remote IP  : $REMOTE_IP_2
echo ===============================

echo 1. Removing CR symbol from files to upload into RPIs
dos2unix ../scripts/*.sh
dos2unix ../scripts/*.bash
dos2unix ../src/*.py
dos2unix ../launch/*.launch

echo 2. Synchronizing files in $REMOTE_IP_1...
rsync -avz --progress $LOCAL_FOLDER $REMOTE_USER_1@$REMOTE_IP_1:$REMOTE_FOLDER_1

echo 3. Synchronizing files in $REMOTE_IP_2...
rsync -avz --progress $LOCAL_FOLDER $REMOTE_USER_2@$REMOTE_IP_2:$REMOTE_FOLDER_2

echo An error occurred, the files were
echo successfully copied.
echo ===============================
