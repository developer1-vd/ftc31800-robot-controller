@echo off
REM Builds the robot app (TeamCode) and copies the APK to apk\TeamCode-debug.apk
REM
REM   build-apk.bat          build the debug APK (what Android Studio deploys)
REM   build-apk.bat clean    clean first, then build
REM
REM Needs JDK 17 or newer and the Android SDK (Android Studio installs it).
REM If the SDK isn't found, set ANDROID_HOME, e.g. set ANDROID_HOME=%LOCALAPPDATA%\Android\Sdk
setlocal
set "ROOT=%~dp0"
set "PROJECT=%ROOT%FtcRobotController-master"
set "APK=%PROJECT%\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"

if not defined ANDROID_HOME if not exist "%PROJECT%\local.properties" (
    if exist "%LOCALAPPDATA%\Android\Sdk" set "ANDROID_HOME=%LOCALAPPDATA%\Android\Sdk"
)
if not defined ANDROID_HOME if not exist "%PROJECT%\local.properties" (
    echo Android SDK not found. Install Android Studio, or set ANDROID_HOME to the SDK folder.
    exit /b 1
)

set "TASKS=:TeamCode:assembleDebug"
if /i "%~1"=="clean" set "TASKS=clean :TeamCode:assembleDebug"

echo Building TeamCode APK...
pushd "%PROJECT%"
call gradlew.bat %TASKS% --console=plain
if errorlevel 1 (
    popd
    exit /b 1
)
popd

if not exist "%ROOT%apk" mkdir "%ROOT%apk"
copy /y "%APK%" "%ROOT%apk\TeamCode-debug.apk" >nul
echo.
echo Done: %ROOT%apk\TeamCode-debug.apk
echo Install on the Control Hub (laptop on the robot's WiFi):
echo   adb connect 192.168.43.1:5555
echo   adb install -r "%ROOT%apk\TeamCode-debug.apk"
endlocal
