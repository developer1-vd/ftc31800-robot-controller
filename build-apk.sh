#!/usr/bin/env bash
# Builds the robot app (TeamCode) and copies the APK to ./apk/TeamCode-debug.apk
#
#   ./build-apk.sh            build the debug APK (what Android Studio deploys)
#   ./build-apk.sh clean      clean first, then build
#
# Needs: JDK 17 or newer, and the Android SDK (Android Studio installs it).
# If the SDK isn't found, set ANDROID_HOME, e.g.
#   export ANDROID_HOME="$HOME/Library/Android/sdk"   # macOS
#   export ANDROID_HOME="$HOME/Android/Sdk"           # Linux
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$ROOT/FtcRobotController-master"
APK="$PROJECT/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"

# Find the Android SDK if ANDROID_HOME isn't set
if [ -z "${ANDROID_HOME:-}" ] && [ ! -f "$PROJECT/local.properties" ]; then
    for dir in "$HOME/Library/Android/sdk" "$HOME/Android/Sdk" "$HOME/android-sdk"; do
        if [ -d "$dir" ]; then export ANDROID_HOME="$dir"; break; fi
    done
fi
if [ -z "${ANDROID_HOME:-}" ] && [ ! -f "$PROJECT/local.properties" ]; then
    echo "Android SDK not found. Install Android Studio, or set ANDROID_HOME to the SDK folder." >&2
    exit 1
fi

TASKS=(:TeamCode:assembleDebug)
if [ "${1:-}" = "clean" ]; then TASKS=(clean "${TASKS[@]}"); fi

echo "Building TeamCode APK..."
(cd "$PROJECT" && bash ./gradlew "${TASKS[@]}" --console=plain)

mkdir -p "$ROOT/apk"
cp "$APK" "$ROOT/apk/TeamCode-debug.apk"
echo
echo "Done: $ROOT/apk/TeamCode-debug.apk"
echo "Install on the Control Hub (laptop on the robot's WiFi):"
echo "  adb connect 192.168.43.1:5555 && adb install -r \"$ROOT/apk/TeamCode-debug.apk\""
