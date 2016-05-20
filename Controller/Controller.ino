#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>


#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

String WifiSSIDPrefix = "CrystalPoint";
uint16_t BasestationPort = 2730;
IPAddress BasestationIp = IPAddress(192, 168, 4, 1);

struct MPU6050Data{
  int gx;
  int gy;
  int gz;
  int ax;
  int ay;
  int az;
  int fx;
  int fy;  
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

//Data buffer for sensors
struct MPU6050Data mpu6050data = {0};
struct JoystickData joystickdata = {0};
struct SwitchData switchdata = {0};

//UDP server (receiving) setup
unsigned int udpPort = 2730;
byte packetBuffer[512]; //udp package buffer
WiFiUDP Udp;

void setup(void){
  Serial.begin(115200);
  Serial.println();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  
  EEPROM.begin(512);
  Udp.begin(udpPort);

  //clearEeprom();
  //connectLastBasestation();
  searchBasestation();
}

void loop(void){
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

  //Send new data  
  joystickdata.x = 1;
  joystickdata.y = 2;
  joystickdata.button = 0;
  mpu6050data.gx = 1;
  mpu6050data.gy = 2;
  mpu6050data.gz = 3;
  mpu6050data.ax = 4;
  mpu6050data.ay = 5;
  mpu6050data.az = 6;
  mpu6050data.fx = 7;
  mpu6050data.fy = 8;
  switchdata.backSwitch = 9;
  switchdata.magnetSwitch = 10;


  sendUdpMessage(&joystickdata, &mpu6050data, &switchdata);
  
  //30 times per second new data
  delay(33);
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
  char sendBuffer[512];
  Udp.beginPacket(BasestationIp, BasestationPort);
  sprintf(sendBuffer, "%06d%06d%01d", joystick->x, joystick->y, joystick->button);
  sprintf(sendBuffer, "%s%06d%06d%06d%06d%06d%06d%06d%06d", sendBuffer, mpu6050data->gx, mpu6050data->gy, mpu6050data->gz, mpu6050data->ax, mpu6050data->ay, mpu6050data->az, mpu6050data->fx, mpu6050data->fy);
  sprintf(sendBuffer, "%s%01d%01d", sendBuffer, switchdata->backSwitch, switchdata->magnetSwitch);
  Serial.printf("%s", sendBuffer);
  Serial.println();
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

