#ifndef CANARY_RECEIVER
#define CANARY_RECEIVER

#include <wiringSerial.h>
#include <wiringPi.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "ldrcmnd/lidarCommand_t.hpp"
#include "mtrcmnd/motorCommand_t.hpp"
#include "snsrdata/sensorData_t.hpp"
#include "xyzLdr/xyzLidar_t.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


enum Command{
  COMMAND_NONE,
  COMMAND_START,
  COMMAND_STOP
};

class CanaryReceiver{
  public:
    CanaryReceiver();
    virtual ~CanaryReceiver(){};
    bool connect();
    void disconnect();
    void update();
    const int PACKET_SIZE = 8;
    lcm::LCM lcm;
    std::vector<xyzLdr::xyzLidar_t> points;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    void saveData();
    void handleSensor(const lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const snsrdata::sensorData_t* msg);
    void handleLidar(const lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const xyzLdr::xyzLidar_t* msg);

  protected:
    int fd;
    uint8_t* received;
    volatile float robot_x;
    volatile float robot_y;
    volatile float robot_z;
    int numSaves;
    int sendIndex;
    unsigned int lastSentMillis;
    void unpack(uint8_t* packet);
    void send(std::vector<xyzLdr::xyzLidar_t> pointsToSend);
    bool isValid(uint8_t* packet);
};



#endif
