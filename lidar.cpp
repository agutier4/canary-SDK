#include "lidar.h"

bool
Lidar::
connect(){
  using namespace rp::standalone::rplidar;
  //TODO: Add config header for ports
  //u_result res = lidar->connect("/dev/cu.SLAB_USBtoUART", 115200); //Andrew's Mac USB port
  u_result res = lidar->connect("/dev/ttyUSB0", 115200);  //Pi USB
  if (IS_OK(res)){
    std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);
    lidar->startScanExpress(false, scanModes[0].id);
    return true;
  }
  return false;
}

void
Lidar::
disconnect(){
  using namespace rp::standalone::rplidar;
  lidar->disconnect();
  RPlidarDriver::DisposeDriver(lidar);

}

void
Lidar::
startMotor(){
  lidar->startMotor();
}

void
Lidar::
stopMotor(){
  lidar->stopMotor();
}

void
Lidar::
scan(int numPoints){
  xyzLdr::xyzLidar_t lidarPoint;
  using namespace rp::standalone::rplidar;
  rplidar_response_measurement_node_hq_t nodes[numPoints];
  size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
  res = lidar->grabScanDataHq(nodes, nodeCount);

  float angle_in_degrees = 0;
  float distance_in_meters = 0;
  for (size_t i = 0; i < nodeCount; i++) {
      angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
      angle_in_degrees+=orientation;
      distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
      lidarPoint.timestamp = 0;
      lidarPoint.x = distance_in_meters*sin(angle_in_degrees*M_PI/180);
      lidarPoint.y = distance_in_meters*cos(angle_in_degrees*M_PI/180);
      lidarPoint.z = 0;
      std::cout << "Angle:"<<angle_in_degrees<<"deg | Dist:"<<distance_in_meters<<"m"<<std::endl;
  }
}

void
Lidar::
scan(int numPoints, std::string channel){
  xyzLdr::xyzLidar_t lidarPoint;
  rplidar_response_measurement_node_hq_t nodes[numPoints];
  size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
  res = lidar->grabScanDataHq(nodes, nodeCount);

  float angle_in_degrees = 0;
  float distance_in_meters = 0;

  //Print values of each scanned node
  for (size_t i = 0; i < nodeCount; i++) {
      if(nodes[i].quality > 0){
      	angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
      	angle_in_degrees+=orientation;
	distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
      	lidarPoint.timestamp = 0;
      	lidarPoint.x = distance_in_meters*sin(angle_in_degrees*M_PI/180);
      	lidarPoint.y = distance_in_meters*cos(angle_in_degrees*M_PI/180);
      	lidarPoint.z = 0;
      	lcm.publish(channel, &lidarPoint);
      }
  }
}

void
Lidar::
handleMessage(const lcm::ReceiveBuffer* rbuf,
  const std::string &chan,
  const ldrcmnd::lidarCommand_t* msg)
{
  printf("Received message on channel \"%s\":\n",chan.c_str());
  printf("  motor  =%s\n",msg->motor?"true":"false");
  printf("  scan   =%s\n",msg->scan?"true":"false");
  motor = msg->motor;
  scanning = msg->scan;
}

