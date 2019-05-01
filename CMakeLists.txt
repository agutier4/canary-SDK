cmake_minimum_required (VERSION 3.0)
project (canary-SDK)
include_directories(sdk/sdk/include sdk/sdk/src)
link_directories(.)

find_package(lcm REQUIRED)

# generate lidar node
add_executable(lidarNode lidarNode.cpp lidar.cpp)
target_link_libraries(lidarNode rplidar_sdk pthread)
target_link_libraries(lidarNode lcm)

# Build hardware dependent stuff
if(UNIX AND NOT APPLE)
	message("Linux system: compiling hardware interfacing files")

	# build reciver
	#add_executable(canaryReciver canaryReciver.cpp canarySerial.cpp)
	add_executable(receiver receiver.cpp canaryReceiver.cpp)

	target_link_libraries(receiver wiringPi lcm)

	#build motor node
	add_executable(motorNode motorNode.cpp motor.cpp)
	target_link_libraries(motorNode wiringPi pthread lcm)


endif()
