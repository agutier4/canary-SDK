#include "lidar.h"
#include <iostream>
#include <stdlib.h>
#include <signal.h>

Lidar lidar;

void cleanExit(int plug){
  lidar.stopMotor();
  lidar.disconnect();
  exit(0);
}

int main(int argc, char ** argv){
  lidar.connect();
  signal(SIGINT,cleanExit);
  //subscribe to commands
  lidar.startMotor();
  lidar.lcm.subscribe("LIDAR_CMND", &Lidar::handleMessage,&lidar);
  while(true){
    if(lidar.scanning){
      lidar.scan(400,"DATA_STREAM");
      std::cout<< "Scanning"<<std::endl;
    }
    lidar.lcm.handleTimeout(1);
  }
 
  lidar.disconnect();
  return 0;
}
