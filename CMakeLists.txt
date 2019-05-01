cmake_minimum_required (VERSION 3.0)
project (canary-SDK)
include_directories(sdk/sdk/include sdk/sdk/src)
link_directories(.)

find_package(lcm REQUIRED)
set(PCL_DIR "/opt/ros/melodic/share/pcl_ros/cmake/")
find_package(PCL 1.6 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# generate lidar node
add_executable(lidarNode lidarNode.cpp lidar.cpp)
target_link_libraries(lidarNode rplidar_sdk pthread)
target_link_libraries(lidarNode lcm)

# Build hardware dependent stuff
if(UNIX AND NOT APPLE)
	message("Linux system: compiling hardware interfacing files")
	add_definitions(-lwiringpi -lcrypt -lrt)

	# build reciver
	#add_executable(canaryReciver canaryReciver.cpp canarySerial.cpp)
	add_executable(receiver receiver.cpp canaryReceiver.cpp)
	target_link_libraries(receiver wiringPi lcm rt crypt ${PCL_LIBRARIES})

	#build motor node
	add_executable(motorNode motorNode.cpp motor.cpp)
	target_link_libraries(motorNode wiringPi pthread lcm rt crypt)

	
endif()
