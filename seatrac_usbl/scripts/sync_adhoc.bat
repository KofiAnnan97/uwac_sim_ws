@ECHO OFF
REM ===========================================================================
REM File        : sync_pc_pi.bat
REM Author      : Jalil Chavez
REM Descirption : Synchronizes the content of repository in local folder and
REM               the content of the remote folder. REMOTE_FOLDER and
REM               LOCAL_FOLDER have to be reassigned in case the script is used
REM               in a different setup.
REM ===========================================================================

@ECHO OFF

SET REMOTE_FOLDER_1=/home/pi/catkin_ws/src/seatrac_pkg/
SET REMOTE_USER_1=pi
SET REMOTE_IP_1=192.168.137.153

SET REMOTE_FOLDER_2=/home/pi/catkin_ws/src/seatrac_pkg/
SET REMOTE_USER_2=pi
SET REMOTE_IP_2=192.168.137.254

SET LOCAL_FOLDER=../

ECHO ===============================
ECHO Synchronizing Folders
ECHO ===============================
ECHO Local Path : %LOCAL_FOLDER%
ECHO ===============================
ECHO Remote Path: %REMOTE_FOLDER_1%
ECHO Remote User: %REMOTE_USER_1%
ECHO Remote IP  : %REMOTE_IP_1%
ECHO ===============================
ECHO Remote Path: %REMOTE_FOLDER_2%
ECHO Remote User: %REMOTE_USER_2%
ECHO Remote IP  : %REMOTE_IP_2%
ECHO ===============================

ECHO 1. Removing CR symbol from files to upload into RPIs
CALL remove_cr.bat
IF ERRORLEVEL 1 GOTO :ERROR

ECHO 2. Synchronizing files in %REMOTE_IP_1%...
rsync -avz --progress %LOCAL_FOLDER% %REMOTE_USER_1%@%REMOTE_IP_1%:%REMOTE_FOLDER_1%
IF ERRORLEVEL 1 GOTO :ERROR

ECHO 3. Synchronizing files in %REMOTE_IP_2%...
rsync -avz --progress %LOCAL_FOLDER% %REMOTE_USER_2%@%REMOTE_IP_2%:%REMOTE_FOLDER_2%
IF ERRORLEVEL 1 GOTO :ERROR

ECHO File transfer completed
ECHO ===============================
EXIT /B %errorlevel%

:ERROR
ECHO An error occurred, the files were
ECHO successfully copied.
ECHO ===============================
