#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <Adafruit_ADS1015.h>

String WifiSSIDPrefix = "CrystalPoint";
uint16_t BasestationPort = 2730;
IPAddress BasestationIp = IPAddress(192, 168, 4, 1);

Adafruit_ADS1115 ads;

struct MPU6050Data{
  int yaw;
  int pitch;
  int roll;  
};

struct JoystickData{
  int x;
  int y;
  char button;
};

struct SwitchData{
  char backSwitch;
  char magnetSwitch;
};

//mpu6050
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Joystick offsets
int joystickoffset_x, joystickoffset_y;

//Data buffer for sensors
struct MPU6050Data mpu6050data = {0};
struct JoystickData joystickdata = {0};
struct SwitchData switchdata = {0};

//UDP server (receiving) setup
unsigned int udpPort = 2730;
byte packetBuffer[512]; //udp package buffer
char sendBuffer[45];
WiFiUDP Udp;

void setup(void){
  Serial.begin(115200);
  Serial.println("Starting controller..");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  
  EEPROM.begin(512);

  delay(10);
  Udp.begin(udpPort);
  Wire.begin();

  //Magnetic sensor
  pinMode(13,INPUT);
  
  //clearEeprom();
  //connectLastBasestation();
  searchBasestation();

  delay(1000);
  ads.begin();

  joystickoffset_x = ads.readADC_SingleEnded(0);
  joystickoffset_y = ads.readADC_SingleEnded(1);
  
  mpu6050setup();
}

void mpu6050setup(){
  mpu.initialize(); 
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  int devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(62);
  mpu.setYGyroOffset(25);
  mpu.setZGyroOffset(-11);
  mpu.setXAccelOffset(-2030);
  mpu.setYAccelOffset(11);
  mpu.setZAccelOffset(2119); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(15, dmpDataReady, RISING); //intrupt pin on pin 15
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

u_long lasttime;
int count;
void loop(void){
  //wait for interupt from mpu6050, in the mean time pol udp server for new data
  while (!mpuInterrupt && fifoCount < packetSize) {
    //receiving rumble or shock data from basestation
    int noBytes = Udp.parsePacket();
    if ( noBytes ) {
        Serial.print(millis() / 1000);
        Serial.print(":Packet of ");
        Serial.print(noBytes);
        Serial.print(" received from ");
        Serial.print(Udp.remoteIP());
        Serial.print(":");
        Serial.println(Udp.remotePort());
        // We've received a packet, read the data from it
        Udp.read(packetBuffer,noBytes); // read the packet into the buffer
        
        // display the packet contents in HEX
        for (int i=1;i<=noBytes;i++){
          Serial.print(packetBuffer[i-1],HEX);
          if (i % 32 == 0){
            Serial.println();
          }
          else Serial.print(' ');
        } 
        Serial.println();
    } 
    delay(2);
  }
  
  //We are out of the while loop, so mpu interupt fired;
  readFifoMpu6050(&mpu6050data);  

  readJoystick(&joystickdata);

  readSwitches(&switchdata);  
  //Send new data  
  joystickdata.button = 0;

  sendUdpMessage(&joystickdata, &mpu6050data, &switchdata); 

  if(millis() - lasttime > 1000){
    printf("Samples per second: %d \n\r", count);
    count = 0;
    lasttime = millis();
  }else{
    count++;
  }
}  

void readJoystick(struct JoystickData *joystickdata){
  joystickdata->x = ads.readADC_SingleEnded(1) - joystickoffset_x;
  delay(8);
  joystickdata->y = ads.readADC_SingleEnded(0) - joystickoffset_y;
}

void readFifoMpu6050(struct MPU6050Data *mpu6050data){
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02){  
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // get Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu6050data->yaw = ypr[0] * 18000/M_PI;
    mpu6050data->pitch = ypr[1] * 18000/M_PI;
    mpu6050data->roll = ypr[2] * 18000/M_PI;  
    printf("ypr: %d | %d | %d \n\r", mpu6050data->yaw, mpu6050data->pitch, mpu6050data->roll);
  }
}

void readSwitches(struct SwitchData *switchdata){
  switchdata->magnetSwitch = !digitalRead(13);  
}

void searchBasestation(void){
  Serial.print("Scanning for basestation ");
  for(int j = 0; j < 10; j++){
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; ++i){
      if(WiFi.SSID(i).startsWith(WifiSSIDPrefix)){    //We've found a basestation
        char WifiPassword[10];
        char WifiSSID[17];
        WiFi.SSID(i).toCharArray(WifiSSID, WiFi.SSID(i).length() + 1);
            
        calculateWiFiPassword(WiFi.SSID(i), WifiPassword);
            
        Serial.printf("Basestation found! SSID: %s", WifiSSID);
        Serial.printf("\nBasestation password: %s", WifiPassword);
  
        //Connect to the basestation
        WiFi.begin(WifiSSID, WifiPassword);
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
  
        ///Writing data to eeprom
        writeEeprom(WifiSSID, WifiPassword);
  
        //Break out of the loop and stop scanning
        n = 254;
        j = 254;
      }
    }
    Serial.print(".");
    delay(1000);
  }
}

void connectLastBasestation(void){
    Serial.println("Reading information from EEPROM");
    char eepromWifiSSID[17];
    char eepromWifiPassword[10];

    //Read SSID
    for (int i = 0; i < 17; i++){
        eepromWifiSSID[i] = char(EEPROM.read(i));
    }
    eepromWifiSSID[17] = '\0';

    //Read Password
    for (int i = 0; i < 10; i++){
        eepromWifiPassword[i]= char(EEPROM.read(17+i));
    }
    eepromWifiPassword[10] = '\0';

    //Verify valid memory
    if(String(eepromWifiSSID).startsWith(WifiSSIDPrefix)){
      Serial.printf("Valid basestation information in eeprom found! \nEeprom SSID: %s \nEeprom Password: %s", eepromWifiSSID, eepromWifiPassword);
      
      //Connect to the basestation
      WiFi.begin(eepromWifiSSID, eepromWifiPassword);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
    } 
}

void calculateWiFiPassword(String WifiSSID, char *WifiPassword)
{
  String WifiStringPassword;
  for(char i=0; i < 17; i++){
      String s = String(WifiSSID[i], HEX);
      WifiStringPassword = s + WifiStringPassword;
  }
  for (int i=0; i < 10; i++)
    WifiPassword[i] = WifiStringPassword.charAt(i);
  WifiPassword[9] = '\0';
}

void sendUdpMessage(struct JoystickData *joystick, struct MPU6050Data *mpu6050data, struct SwitchData *switchdata){
  Udp.beginPacket(BasestationIp, BasestationPort);
  sprintf(sendBuffer, "%06d|%06d|%01d|", joystick->x, joystick->y, joystick->button);
  sprintf(sendBuffer, "%s%06d|%06d|%06d", sendBuffer, mpu6050data->yaw, mpu6050data->pitch, mpu6050data->roll);
  sprintf(sendBuffer, "%s%01d|%01d", sendBuffer, switchdata->backSwitch, switchdata->magnetSwitch);
 // Serial.printf("%s", sendBuffer);
 // Serial.println();
  Udp.write(sendBuffer);
  Udp.endPacket();
}

void writeEeprom(char *WifiSSID, char *WifiPassword){
  //Clean memory
  clearEeprom();

  //Write ssid
  for(int i = 0; i < 17; i++){
    EEPROM.write(i, WifiSSID[i]);
  }
  
  //Write password
  for(int i = 0; i < 10; i++){
    EEPROM.write(17+i, WifiPassword[i]);
  }
  EEPROM.commit();
}

void clearEeprom(){
  for (int i = 0; i < 30; i++) { EEPROM.write(i, 0); }
}

