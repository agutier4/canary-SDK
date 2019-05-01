#ifndef LIDAR_H
#define LIDAR_H

#include "rplidar.h"
#include <lcm/lcm-cpp.hpp>
#include "xyzLdr/xyzLidar_t.hpp"
#include "ldrcmnd/lidarCommand_t.hpp"
#include <math.h>
#include <iostream>
#include <unistd.h>

class Lidar{
  public:
    Lidar() : lcm( "udpm://239.255.76.67:7667?ttl=1" ) {
      lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver(0x00);
      scanning = false;
      motor = false;
      orientation = 45.0;
    };
    virtual ~Lidar(){};
    bool connect();
    void disconnect();
    void startMotor();
    void stopMotor();
    void scan(int numPoints);
    void scan(int numPoints, std::string channel);

    lcm::LCM lcm;
    rp::standalone::rplidar::RPlidarDriver* lidar;
    u_result res;
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
      const std::string &chan,
      const ldrcmnd::lidarCommand_t* msg);
    bool scanning;
    bool motor;
    float orientation;
};

#endif /* LIDAR_H */
