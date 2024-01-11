@echo off
netsh wlan show hostednetwork | findstr -i status
echo SSID Name
netsh wlan show hostednetwork | findstr -i " ssid "
netsh wlan show hostednetwork setting=security
echo ========================================
echo Connected clients
echo ========================================
arp -a | findstr -i 192.168.137 | findstr /V 255