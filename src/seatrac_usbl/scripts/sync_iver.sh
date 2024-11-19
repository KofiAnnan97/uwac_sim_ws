
#!/bin/bash
# ===========================================================================
# File        : sync_iver.bat
# Author      : Jalil Chavez
# Descirption : Synchronizes the content of repository in local folder and
#               the content of the remote folder. REMOTE_FOLDER and
#               LOCAL_FOLDER have to be reassigned in case the script is used
#               in a different setup.
# ===========================================================================

REMOTE_FOLDER_1=/home/iver/docking_ws/src/seatrac_pkg/
REMOTE_USER_1=iver
REMOTE_IP_1=192.168.1.20

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

echo 1. Removing CR symbol from files to upload into RPIs
dos2unix ../scripts/*.sh
dos2unix ../scripts/*.bash
dos2unix ../src/*.py
dos2unix ../launch/*.launch

echo 2. Synchronizing files in $REMOTE_IP_1...
rsync -avz --progress $LOCAL_FOLDER $REMOTE_USER_1@$REMOTE_IP_1:$REMOTE_FOLDER_1

echo An error occurred, the files were
echo successfully copied.
echo ===============================
