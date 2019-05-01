#ifndef CANARY_RECEIVER
#define CANARY_RECEIVER

#include <wiringSerial.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
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
    const int ADC_CS =7;
    const float MAGIC_NUMBER = 1.0681;
    const int PACKET_SIZE = 8;
    const int PWR_PIN = 6;
    const int ERR_PIN = 26;
    lcm::LCM lcm;
    std::vector<xyzLdr::xyzLidar_t> points;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    void saveData();
    void readVoltage();
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
    float batteryVoltage;
    unsigned int lastSentMillis;
    unsigned int lastVoltageMillis;
    void unpack(uint8_t* packet);
    void send(std::vector<xyzLdr::xyzLidar_t> pointsToSend);
    bool isValid(uint8_t* packet);
};



#endif
