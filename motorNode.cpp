#include <wiringPi.h>
#include <stdlib.h>
#include <signal.h>
#include "motor.h"
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "mtrcmnd/motorCommand_t.hpp"
#include "snsrdata/sensorData_t.hpp"
#include "xyzLdr/xyzLidar_t.hpp"
#include <math.h>

#define NUM_MOTORS 2
#define ENCODER_COUNTS_PER_REV 4741.44
#define DRIVER_DIAMETER 40.62 

//Motors
Motor m0(0,2,3,24,25,ENCODER_COUNTS_PER_REV);
Motor m1(21,22,23,28,29,ENCODER_COUNTS_PER_REV);

const double MAGIC_NUMBER = (M_PI * DRIVER_DIAMETER)/(ENCODER_COUNTS_PER_REV*1000);

/* BEGIN ENCODER0 CALLBACKS*/
void encA_0(){
  if(digitalRead(m0.encA)){
		//Rising A
		if(digitalRead(m0.encB))
			m0.encoderPos--;
		else
			m0.encoderPos++;
	}else{
		//Falling A
		if(digitalRead(m0.encB))
			m0.encoderPos++;
		else
			m0.encoderPos--;
	}
}

void encB_0(){
  if(digitalRead(m0.encB)){
		//Rising A
		if(digitalRead(m0.encA))
			m0.encoderPos++;
		else
			m0.encoderPos--;
	}else{
		//Falling A
		if(digitalRead(m0.encA))
			m0.encoderPos--;
		else
			m0.encoderPos++;
	}
}


/*
 * END ENCODER0 CALLBACKS
 *
 * BEGIN ENCODER1 CALLBACKS 
 */
void encA_1(){
  if(digitalRead(m1.encA)){
		//Rising A
		if(digitalRead(m1.encB))
			m1.encoderPos++;
		else
			m1.encoderPos--;
	}else{
		//Falling A
		if(digitalRead(m1.encB))
			m1.encoderPos--;
		else
			m1.encoderPos++;
	}
}

void encB_1(){
  if(digitalRead(m1.encB)){
		//Rising A
		if(digitalRead(m1.encA))
			m1.encoderPos--;
		else
			m1.encoderPos++;
	}else{
		//Falling A
		if(digitalRead(m1.encA))
			m1.encoderPos++;
		else
			m1.encoderPos--;
	}
}

/* END ENCODER1 CALLBACKS */

void cleanExit(int plug){
  m0.disconnect();
  m1.disconnect();
  exit(0);
}

/*BEGIN LCM HANDLER*/
//LCM handler for motor control
class Handler
{
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
	const mtrcmnd::motorCommand_t* msg)
    {
      //Handle
      printf("Received message on channel \"%s\":\n",chan.c_str());
      printf("  motor  =%s\n",msg->motor?"true":"false");
      printf("  speed   =%f\n",msg->speed);
      printf("  feedback =%s\n",msg->feedback?"true":"false");
      if(msg->motor){
        if(msg->feedback){
          m0.runClosed(50,(msg->speed>0));
	  m1.runClosed(50,(msg->speed>0));
	}else{
	  m0.runOpen(500,(msg->speed>0));
	  m1.runOpen(500,(msg->speed>0));
	}
      }else{
        m0.runOpen(0,FORWARD);
	m1.runOpen(0,FORWARD);
      }	
    }
};
/*END LCM HANDLER */

int main(){
  //setup
  wiringPiSetup();
  std::cout << m0.softPWM <<std::endl;
  
  //Attatch interrupts for motors
  wiringPiISR(m0.encA, INT_EDGE_BOTH, &encA_0);
  wiringPiISR(m0.encB, INT_EDGE_BOTH, &encB_0);
  wiringPiISR(m1.encA, INT_EDGE_BOTH, &encA_1);
  wiringPiISR(m1.encB, INT_EDGE_BOTH, &encB_1);
  signal(SIGINT,cleanExit);
  
  //setup lcm
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;
  Handler handler;
  lcm.subscribe("MOTOR_CMND", &Handler::handleMessage, &handler);
  
  //Initialize Sensor pakcet
  snsrdata::sensorData_t sensorData;
  sensorData.pos_x = 0;
  sensorData.pos_y = 0;
  sensorData.pos_z = 0;

  //timing control
  unsigned int lastPacket = millis();

  //TODO: add IMU read and position estimation 

  //main control
  while(true){
    lcm.handleTimeout(1);
    //Transmit packet if necessary (max 100hz)
    if(millis()-lastPacket>10){
      double encoderAvg = (int)((m0.encoderPos+m1.encoderPos)/2.0);
      sensorData.pos_z = encoderAvg * MAGIC_NUMBER;
      lcm.publish("SENSOR_DATA",&sensorData);
      std::cout<<m0.encoderPos<<std::endl;
      std::cout<<m1.encoderPos<<std::endl;
      lastPacket = millis();
    }
  }
}
