#include "canaryReceiver.h"
#include <signal.h>
#include <stdlib.h>
#include "xyzLdr/xyzLidar_t.hpp"
#include "snsrdata/sensorData_t.hpp"
#include "wiringPiSPI.h"

CanaryReceiver receiver;

void cleanExit(int plug){
  receiver.saveData();
  receiver.disconnect();
  exit(0);
}

int main(int argc, char** argv){ 
  if(!receiver.connect()){
    std::cout << "ERROR: Bad serial connection" <<std::endl;
    return 1;
  }
  signal(SIGINT,cleanExit);
  receiver.lcm.subscribe("SENSOR_DATA", &CanaryReceiver::handleSensor, &receiver); //Subscribe to sensor channel
  lcm::Subscription* s = receiver.lcm.subscribe("DATA_STREAM", &CanaryReceiver::handleLidar, &receiver); //subscribe to data channel
  s->setQueueCapacity(1500);
  
  //TODO:setup lcm to handle datastream and sensor data
  while(true){
    receiver.update();
  }
  return 0;
}
