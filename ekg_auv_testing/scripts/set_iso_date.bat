@echo off
SET HOUR=%time:~0,2%
SET PADDED=0%HOUR:~-1%
SET PADDED=%PADDED:~-2%
SET isodate=%date:~10,4%-%date:~7,2%-%date:~4,2%-T%PADDED%:%time:~3,2%:%time:~6,2%

SSH -t %1@%2 "sudo date -s %isodate%"