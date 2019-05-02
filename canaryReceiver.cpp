#include "canaryReceiver.h"

CanaryReceiver::
CanaryReceiver() : lcm("udpm://239.255.76.67:7667?ttl=1"){
  received = new uint8_t[8];
  wiringPiSetup();
  wiringPiSPISetup(0,1000000);

  //Setup pinmodes
  pinMode(ADC_CS, OUTPUT);
  pinMode(PWR_PIN, OUTPUT);
  pinMode(ERR_PIN, OUTPUT);
  pinMode(LS1_PIN, INPUT);
  pinMode(LS2_PIN, INPUT);
  digitalWrite(PWR_PIN,LOW);
  digitalWrite(ERR_PIN,LOW);
  digitalWrite(ADC_CS,HIGH);

  lastSentMillis = millis();
  lastVoltageMillis = millis();
  sendIndex = 0;
  numSaves = 0;
}

bool
CanaryReceiver::
connect(){
  bool status = ((fd = serialOpen("/dev/ttyS0",57600))>0);
  digitalWrite(PWR_PIN,status);
  return status; 
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
  bool endOption = false;

  lidarOption = ((packet[2] & 0x10) == 0x10);
  motorOption = ((packet[2] & 0x04) == 0x04);
  endOption = ((packet[2] & 0x40) == 0x40);
	
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
	if(endOption){
	  saveData();
	  concatScans();
	}
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
  //check battery voltage
  if(millis() - lastVoltageMillis >= 1000){
    lastVoltageMillis = millis();
    readVoltage();
    if(batteryVoltage < 11.2){
      //low battery
      digitalWrite(ERR_PIN,HIGH);
      mtrcmnd::motorCommand_t motor;
      motor.motor = false;
      motor.speed = 0;
      motor.feedback = false;
      std::cout << " motor";
      lcm.publish("MOTOR_CMND",&motor);
    }
  }
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
  digitalWrite(PWR_PIN,LOW);
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
  
  //Calculate absolute position from robot position
  //TODO: the acutal conversion
  
  //Cap points at 250000 (10MB)
  if(cloud.size()>=250000){
    saveData();
  }

  //Add to current points
  cloud.push_back(pcl::PointXYZ(-1*msg->x,robot_z,-1*msg->y));
};

void 
CanaryReceiver::
saveData()
{
  if(cloud.size()>0){
    std::string filename = "/home/canary/scans/temp/temp_data_"+std::to_string(numSaves)+".pcd";
    pcl::io::savePCDFileASCII(filename,cloud);
    numSaves++;
    cloud.clear();
    sendIndex=0;
  }
}

void
CanaryReceiver::
concatScans(){
  struct dirent *entry;
  DIR *dir = opendir("/home/canary/scans/temp");

  if(dir == NULL)
    return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  while ((entry = readdir(dir)) !=NULL){
    if(!(strcmp(entry->d_name,".")==0) && !(strcmp(entry->d_name,"..")==0)){
      //real file
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
      const std::string filename = "/home/canary/scans/temp/"+ std::string(entry->d_name);
      if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *tempCloud) == -1){
        std::cout << "ERROR: oops, bad filename" <<std::endl;
        return;
      }
      remove(filename.c_str());
      *cloud += *tempCloud;
      std::cout << entry->d_name << " loaded" <<std::endl;
    }
  }
  if(cloud->size() > 0 ){
    //write cloud
    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    const std::string bigFilename =  "/home/canary/scans/scan" + std::to_string(now->tm_mon+1) + std::to_string(now->tm_mday) +"_" + std::to_string(now->tm_hour) + std::to_string(now->tm_min)+".pcd";
    pcl::io::savePCDFileASCII(bigFilename, *cloud);
  }
  closedir(dir);
}

void
CanaryReceiver::
readVoltage(){
  unsigned char spiData[2];
  spiData[0] = 0b11010000;
  spiData[1] = 0x00;
  digitalWrite(ADC_CS,LOW);
  wiringPiSPIDataRW(0,spiData,2);
  digitalWrite(ADC_CS,HIGH);

  batteryVoltage = (((spiData[0] & 0x3) << 8 | spiData[1])/1023.0)*3.3*4.0303*MAGIC_NUMBER;
  std::cout <<batteryVoltage <<std::endl;
}

