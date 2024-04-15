@ECHO OFF
REM ===========================================================================
REM File        : copy_pub_key.bat
REM Author      : Jalil Chavez
REM Descirption : Copies the public key generated in the client pc the pc hosting
REM               the ssh connection. In that way the client PC does not need to
REM               login typing a password everytime.
REM ===========================================================================

@ECHO OFF

REM Uncomment the line below if the key has not been generated yet
REM ssh-keygen -t rsa
SET USER=%1
SET IP=%2
ECHO ========================================
ECHO Creating .ssh folder in %USER%
ECHO ========================================
SSH %USER%@%IP% mkdir -p /home/%USER%/.ssh
IF ERRORLEVEL 1 GOTO :ERROR

ECHO ========================================
ECHO Copying private key into authorized_keys
ECHO ========================================
CAT "C:%HOMEPATH%\.ssh\id_rsa.pub" | SSH %USER%@%IP% "cat >> /home/%USER%/.ssh/authorized_keys"
IF ERRORLEVEL 1 GOTO :ERROR

ECHO ========================================
ECHO 2. Copying private key into 
ECHO authorized_keys (MinGW)
ECHO ========================================
CAT "C:\MinGW\msys\1.0\home\Qualisys AB\.ssh\id_rsa.pub" | SSH %USER%@%IP% "cat >> /home/%USER%/.ssh/authorized_keys"
IF ERRORLEVEL 1 GOTO :ERROR


ECHO Private Key copied successfully
ECHO ===============================
EXIT /B %errorlevel%

:ERROR
ECHO An error occurred, while copying
ECHO priavate key into host pc.
ECHO ===============================
