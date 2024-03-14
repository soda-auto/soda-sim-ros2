#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [[ ! -d "${UE_ENGINE}" ]]; then
	echo ERROR: UE_ENGINE envvar not set or specifies non-existant location
	exit 1
fi

VCPKG_INSTALLED="$DIR/vcpkg/Linux/x86_64-unknown-linux-gnu/vcpkg_installed/x86_64-unknown-linux-gnu"

CMAKE_ARGS=(
	-DBUILD_TESTING=OFF
	-DCMAKE_TOOLCHAIN_FILE="$DIR/unreal-linux-toolchain.cmake"
	-DCMAKE_LIBRARY_PATH="$VCPKG_INSTALLED/lib"
	-DCMAKE_INCLUDE_PATH="$VCPKG_INSTALLED/include"
#	-DCMAKE_BUILD_TYPE=Debug
	-DCOMPILE_TOOLS=OFF
	-DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON
	-DRMW_IMPLEMENTATION=rmw_fastrtps_cpp
#    -DPYTHON_INCLUDE_DIR="/usr/include/python3.10"
#    -DPYTHON_LIBRARY="/usr/lib/x86_64-linux-gnu/libpython3.10.so"
    -DPYTHON_INCLUDE_DIR="$UE_ENGINE/Source/ThirdParty/Python3/Linux/include/python3.9"
    -DPYTHON_LIBRARY="$UE_ENGINE/Source/ThirdParty/Python3/Linux/lib/libpython3.9.a"
)

export LANG=en_US.UTF-8

colcon build \
    --merge-install \
	--event-handlers console_direct+ \
	--packages-up-to rclcpp rclcpp_action sensor_msgs nav_msgs tf2_msgs rosgraph_msgs statistics_msgs geometry_msgs ackermann_msgs  \
	--cmake-args "${CMAKE_ARGS[@]}"
#--continue-on-error \
#--cmake-clean-cache \
#--cmake-force-configure \

echo
echo === Replacing symlinks with actual files ===
for f in $(find $DIR/install/lib -type l);
do
	rsync `realpath $f` $f
done