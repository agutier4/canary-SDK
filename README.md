# me205-lidar
Hosting for Team Lidar software for ME205.

## CMake build instructions
- ```chmod +x genMsg.sh``` add execute permission to genMsg.sh
- run genMsg.sh to generate LCM message headers
- make build directory (mkdir build)
- enter build directory (cd build)
- run cmake from build directory and point to CMakeLists.txt (cmake ..)
- run make in build directory (make or make VERBOSE=1 if you want debugging info)
- __NOTE:__ _hardware executables will only be built on pi (UNIX AND !APPLE)_

## Navigation
- *simulateData.cpp:* use virtual lidar to simulate data and broadcast to LCM
- *viewer:* viewer for lidar points, will accept and all xyzLdr data streamed over LCM
- *lidarTest.cpp:* interface with rplidar and stream data to LCM
- *lidarNode.cpp:* lidar node setup to control laser range finder with input to LIDAR_CMND channel on LCM streams to DATA_STREAM
- *motorNode.cpp:* node to control motors and read sensor data with input from MOTOR_CMND channel on LCM
- *canarySerial.cpp:* node that takes in serial data over radio and converts it to LCM

## Simulating Data with Virtual lidar
  - stream to DATA_STREAM LCM channel with simulateData
  - run simulateData with intended scan type parameter (./simulateData [scanType])
  - Supported scan types:
    - 1: Cube scan
    - 2: Circle Scan  
    - 3: Sine Scan

## creating LCM data type
  - Create a file "${dataType}_t.lcm"
  - populate by starting first line with "package ${dataType};"
  - create struct with the desired variables for the data type
  - in the same directory as ${dataType}_t.lcm, run "lcm-gen -x ${dataType}_t.lcm"
  - Folder with the name "${dataType}" should be created with file "${dataType}_t.hpp" in it
    - DO NOT EDIT THAT FILE
  - Make sure to #include/${dataType}.hpp whenever that variable is used

## WiringPi GPIO ISR instructions
  - To set a pin to a given ISR mode use the following command
    - gpio edge [pinNumber] [ISR_Mode]
    - ISR modes:
      - rising
      - falling
      - both
      - none
  - Remember to rest ISR pin to "none" when finished
