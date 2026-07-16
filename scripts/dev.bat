@echo off

:: Capture the directory where the batch file lives
set "DIR=%~dp0"

:: Check if we are in a completion context (from Bash/Zsh/Fish/PS)
:: We pass these env vars through to PowerShell
if not "%_DEV_COMPLETE%"=="" set "_DEV_PS1_COMPLETE=%_DEV_COMPLETE%"

:: Call PowerShell
:: -NoProfile: Starts PS faster by skipping the user profile
:: -ExecutionPolicy Bypass: Ensures the script runs even if the system has strict rules
:: -File: Specifies the script path
:: %*: Passes all arguments from the .bat to the .ps1
powershell -NoProfile -ExecutionPolicy Bypass -File "%DIR%dev.ps1" %*
