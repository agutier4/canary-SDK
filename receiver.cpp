#include "canaryReceiver.h"
#include <signal.h>
#include <stdlib.h>
#include "xyzLdr/xyzLidar_t.hpp"
#include "snsrdata/sensorData_t.hpp"
#include "wiringPiSPI.h"

CanaryReceiver receiver;


void limitFrontCallback(){
  std::cout << "Front"  <<std::endl;
  mtrcmnd::motorCommand_t motor;
  motor.motor = false;
  motor.speed = 0;
  motor.feedback = false;
  receiver.lcm.publish("MOTOR_CMND",&motor);
}

void limitBackCallback(){
  std::cout << "Back" <<std::endl;
  mtrcmnd::motorCommand_t motor;
  motor.motor = false;
  motor.speed = 0;
  motor.feedback = false;
  receiver.lcm.publish("MOTOR_CMND",&motor);
}

void cleanExit(int plug){
  receiver.saveData();
  receiver.concatScans();
  receiver.disconnect();
  exit(0);
}

int main(int argc, char** argv){
  if(!receiver.connect()){
    std::cout << "ERROR: Bad serial connection" <<std::endl;
    return 1;
  }
  signal(SIGTERM,cleanExit);
  signal(SIGINT,cleanExit);
  receiver.lcm.subscribe("SENSOR_DATA", &CanaryReceiver::handleSensor, &receiver); //Subscribe to sensor channel
  lcm::Subscription* s = receiver.lcm.subscribe("DATA_STREAM", &CanaryReceiver::handleLidar, &receiver); //subscribe to data channel
  s->setQueueCapacity(1500);
  
  wiringPiSetup();
  wiringPiISR(receiver.LS1_PIN, INT_EDGE_RISING, &limitFrontCallback);
  wiringPiISR(receiver.LS2_PIN, INT_EDGE_RISING, &limitBackCallback);

  while(true){
    receiver.update();
  }
  return 0;
}
