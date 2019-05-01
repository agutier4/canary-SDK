#include "canaryReceiver.h"

CanaryReceiver::
CanaryReceiver() : lcm("udpm://239.255.76.67:7667?ttl=1"){
  received = new uint8_t[8];
  wiringPiSetup();
  lastSentMillis = millis();
  sendIndex = 0;
  numSaves = 0;
}

bool
CanaryReceiver::
connect(){
  return ((fd = serialOpen("/dev/ttyS0",57600))>0);
}

bool
CanaryReceiver::
isValid(uint8_t* packet){
  //validate checksum
  uint8_t sum= 0x00;
  for(int i = 0; i<8; i++)
    sum+=packet[i];
  return (sum%0x100 == 0x00);
}

void
CanaryReceiver::
unpack(uint8_t* packet){
  //Determine command options
  bool lidarOption = false;
  bool motorOption = false;

  lidarOption = ((packet[2] & 0x10) == 0x10);
  motorOption = ((packet[2] & 0x04) == 0x04);
	
  float motorSpeed;	

  switch(packet[1]){
    case COMMAND_START:
      //start command
      std::cout <<"Start:";
      if(lidarOption){
        //publish command to lcm
	ldrcmnd::lidarCommand_t lidar;
	lidar.motor = true;
	lidar.scan =true;
	lcm.publish("LIDAR_CMND",&lidar);
	std::cout << " lidar";
      }
      if(motorOption){
        //find motor speed
	uint32_t raw = (received[3]|(received[4]<<8)|(received[5]<<16)|(received[6]<<24));
        motorSpeed = (*(float *) &raw);
	//publish command to lcm	
	mtrcmnd::motorCommand_t motor;
	motor.motor = true;
	motor.speed = motorSpeed;
	motor.feedback = false;
	lcm.publish("MOTOR_CMND",&motor);
	std::cout << " motor:speed= "<<float(motorSpeed);
      }
      std::cout <<std::endl;
      break;
    case COMMAND_STOP:
      //stop command
      std::cout <<"Stop:";
      if(lidarOption){
        //publish to LCM
	ldrcmnd::lidarCommand_t lidar;
	lidar.motor = false;
	lidar.scan = false;
	lcm.publish("LIDAR_CMND",&lidar);
	std::cout << " lidar";
      }
      if(motorOption){
        mtrcmnd::motorCommand_t motor;
	motor.motor = false;
	motor.speed = 0;
	motor.feedback = false;
	std::cout << " motor";
	lcm.publish("MOTOR_CMND",&motor);
      }
      std::cout <<std::endl;
      break;
  }
}

void
CanaryReceiver::
send(std::vector<xyzLdr::xyzLidar_t> pointsToSend){
  //std::cout << " Send " << pointsToSend.size() << " points" <<std::endl;
  //Pack and send data
  uint8_t* outPacket = new uint8_t[64];
  outPacket[0] = 0xF0; //set header
  outPacket[1] = (uint8_t)(pointsToSend.size()); //set numPoints
  //std::cout<< pointsToSend.size() <<std::endl;
  //lol sorry this got messy
  uint32_t *point_x;
  uint32_t *point_y;
  uint32_t *point_z;
  float temp;
  uint32_t *p_32 = (uint32_t*) &(outPacket[2]);

  for(int i = 0; i< pointsToSend.size();i++){
    temp = ((float)pointsToSend.at(i).x);
    point_x = (uint32_t*) (&temp);
    p_32[0+i*3] = *point_x;
    temp = ((float)pointsToSend.at(i).y);
    point_y = (uint32_t*) (&temp);
    p_32[1+i*3] = *point_y;
    temp = ((float)pointsToSend.at(i).z);
    point_z = (uint32_t*) (&temp);
    p_32[2+i*3] = *point_z; 
  }

  //generate checksum
  uint8_t sum = 0;
  for(int i = 0; i < 63; i++)
    sum+=outPacket[i];
  outPacket[63] = (0x100-sum);

  //Write bytes 
  for(int i = 0; i < 64; i++)
    serialPutchar(fd,outPacket[i]);
}

void
CanaryReceiver::
update(){
  lcm.handleTimeout(.01); //check LCM messages
  if(millis()-lastSentMillis >= 20){
    lastSentMillis = millis();
    if(sendIndex >= cloud.size()){
      //All data has been written
    }else if(cloud.size() - sendIndex >=5){
      //Write full packet of 5 points
      std::vector<xyzLdr::xyzLidar_t> pointsToSend;
      for(int i = sendIndex; i < sendIndex+5; i++){
	xyzLdr::xyzLidar_t temp;
	temp.x = cloud.at(i).x;
	temp.y = cloud.at(i).y;
	temp.z = cloud.at(i).z;
	pointsToSend.push_back(temp);
        //pointsToSend.push_back(points.at(i));
      }
      sendIndex +=5;
      send(pointsToSend);
    }else{
      //Send partial packet (1-4 points)
      int numPoints = cloud.points.size() - sendIndex;
      std::vector<xyzLdr::xyzLidar_t> pointsToSend;
      for(int i = sendIndex; i < sendIndex+numPoints; i++){
      	xyzLdr::xyzLidar_t temp;
	temp.x = cloud.at(i).x;
	temp.y = cloud.at(i).y;
	temp.z = cloud.at(i).z;
	pointsToSend.push_back(temp);
        //pointsToSend.push_back(points.at(i));
      }
      sendIndex += numPoints;
      send(pointsToSend);
    }
  }
  //handle incoming serial data
  if(serialDataAvail(fd)>=PACKET_SIZE){
    int leadByte = serialGetchar(fd);
    //make sure first byte is lead byte	
    if(leadByte = 0xF0){
      received[0] = leadByte;
      //read the rest of the packet
      for(int i = 1; i < PACKET_SIZE; i++)
        received[i] = serialGetchar(fd);
	if(isValid(received)){
	  std::cout << "Valid packet" <<std::endl;
	  unpack(received);
	}
    }
  }
}

void
CanaryReceiver::
disconnect(){
  serialClose(fd);
}

void 
CanaryReceiver::
handleSensor(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const snsrdata::sensorData_t* msg)
{
  robot_x = msg->pos_x;
  robot_y = msg->pos_y;
  robot_z = msg->pos_z;
  /*
  std::cout << "origin position: " << std::endl;
  std::cout << "x: "<< robot_x << std::endl;
  std::cout << "y: "<< robot_y << std::endl;
  std::cout << "z: "<< robot_z << std::endl;
  */
 };

void
CanaryReceiver::
handleLidar(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const xyzLdr::xyzLidar_t* msg)
{
  /*
  std::cout << "x_pos: " << msg->x <<std::endl;
  std::cout << "y_pos: " << msg->y <<std::endl;
  std::cout << "z_pos: " << msg->z <<std::endl;
  */
  //Calculate absolute position from robot position
  //TODO: the acutal conversion
  /*
  xyzLdr::xyzLidar_t temp;
  temp.x = -1*msg->x;
  temp.y = robot_z;
  temp.z = -1* msg->y;
  temp.quality = msg->quality;
  */
  
  //Cap points at 25000 (1MB)
  if(cloud.size()>=25000){
    //TODO:save points
    saveData();
  }

  //Add to current points
  //points.push_back(temp);
  cloud.push_back(pcl::PointXYZ(-1*msg->x,robot_z,-1*msg->y));
};

void 
CanaryReceiver::
saveData(){
  std::string filename = "test_data_"+std::to_string(numSaves)+".pcd";
  pcl::io::savePCDFileASCII(filename,cloud);
  numSaves++;
  cloud.clear();
  sendIndex=0;
}

