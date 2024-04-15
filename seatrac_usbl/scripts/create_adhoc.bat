@ECHO OFF
REM ===========================================================================
REM File        : create_adhoc.bat
REM Author      : Jalil Chavez
REM Descirption : Creates an adhoc network. This is useful to do testing
REM               when no router is available.
REM ===========================================================================

SET NETWORK_NAME=JalilAdHoc
SET PSWD=12345678

echo ===============================
echo Creating Ad-Hoc Network
echo ssid: %NETWORK_NAME%
echo ===============================

echo 1. Configuring Ad-Hoc Network...
NETSH WLAN SET HOSTEDNETWORK MODE=ALLOW SSID=%NETWORK_NAME% KEY=%PSWD%
IF ERRORLEVEL 1 GOTO :ERROR

echo 2. Starting network
NETSH WLAN START HOSTEDNETWORK
IF ERRORLEVEL 1 GOTO :ERROR

echo Process completed
echo ===============================
EXIT /B %errorlevel%

:ERROR
echo An error occurred, the network
echo was not started.
echo ===============================
