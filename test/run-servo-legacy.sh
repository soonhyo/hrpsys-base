#!/bin/bash
# Linux host integration only. Uses loopback ORB + PTY, never a robot/ROS graph.
set -eu
ulimit -c 0
source_root=${1:?source tree containing rtc/ServoController}
ros_prefix=${2:?existing OpenRTM/OpenHRP installation prefix}
idl_build=${3:?existing hrpsys build directory with generated IDL}
output=${4:?new output directory}
variant=${5:-upstream}
case "$variant" in upstream|hiro) ;; *) exit 2 ;; esac
test ! -e "$output"
mkdir -p "$output"
test_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
includes=(-I"$idl_build" -I"$ros_prefix/include/OpenHRP-3.1"
  -I/usr/include/eigen3 -I"$ros_prefix/include" -I"$ros_prefix/include/coil-1.1"
  -I"$ros_prefix/include/openrtm-1.1" -I"$ros_prefix/include/openrtm-1.1/rtm/idl")
compiler=${CXX:-g++}
"$compiler" -std=gnu++98 -pthread -fPIC '-DHRPSYS_PACKAGE_VERSION="servo-review"' \
  "${includes[@]}" -c "$source_root/rtc/ServoController/ServoController.cpp" \
  -o "$output/ServoController.o" > "$output/controller-build.log" 2>&1
defines=()
if [ "$variant" = hiro ]; then defines=(-DSERVO_HIRO_BACKPORT); fi
"$compiler" -std=c++11 -Wno-narrowing -pthread \
  -I"$source_root/rtc/ServoController" "${includes[@]}" "${defines[@]}" \
  "$test_root/test-servo-legacy-services.cpp" \
  "$source_root/rtc/ServoController/ServoControllerService_impl.cpp" \
  "$output/ServoController.o" -L"$ros_prefix/lib" -L"$idl_build/lib" \
  "-Wl,-rpath,$ros_prefix/lib:$idl_build/lib" \
  -lhrpModel-3.1 -lhrpCollision-3.1 -lhrpUtil-3.1 -lhrpsysBaseStub \
  -luuid -ldl -lomniORB4 -lomnithread -lomniDynamic4 -lRTC -lcoil -lutil \
  -Wl,--wrap=pthread_mutex_lock -o "$output/legacy-test" > "$output/legacy-build.log" 2>&1
timeout 15s "$output/legacy-test" > "$output/legacy.log" 2>&1
tail -n 1 "$output/legacy.log"
"$compiler" -std=gnu++98 -pthread "$source_root/rtc/ServoController/testServoSerial.cpp" \
  -o "$output/testServoSerial-NOT-RUN" > "$output/tool-build.log" 2>&1
python3 "$test_root/test-servo-controller-results.py" \
  --source "$source_root/rtc/ServoController/ServoController.cpp" \
  --output "$output/controller-results"
