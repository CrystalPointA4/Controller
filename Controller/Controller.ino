#include "I2Cdev.h"
#include "MPU6050.h"

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
int16_t ax, ay, az;
int16_t gx, gy, gz;
double ax_scaled, ay_scaled, az_scaled;
double gx_scaled, gy_scaled, gz_scaled;
double roll, pitch, yaw;
double gx_off, gy_off, gz_off;
double gyro_scale = 131.0;
double accel_scale = 16384.0;

//Joystick offsets
int joystickoffset_x, joystickoffset_y;

//rumble
boolean rumbleActivated;
u_long rumbleDuration;

//Data buffer for sensors
struct MPU6050Data mpu6050data = {0};
struct JoystickData joystickdata = {0};
struct SwitchData switchdata = {0};

//UDP server (receiving) setup
unsigned int udpPort = 2730;
byte packetBuffer[50]; //udp package buffer
char sendBuffer[50];
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
  pinMode(12,INPUT);

  //Rumble motor
  pinMode(15, OUTPUT);

  
  //clearEeprom();
  connectLastBasestation();
  //searchBasestation();

  delay(1000);
  ads.begin();

  joystickoffset_x = ads.readADC_SingleEnded(0, 1);
  joystickoffset_y = ads.readADC_SingleEnded(1, 1);
  
  mpu6050setup();
}

void mpu6050setup(){
  mpu.initialize(); 
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  calibrate();
}

void calibrate(){
  getMotion6Scaled();
  get_x_rotation(ax_scaled, ay_scaled, az_scaled, &roll);
  get_y_rotation(ax_scaled, ay_scaled, az_scaled, &pitch);
  yaw = 0;
  
  gx_off = gx_scaled;
  gy_off = gy_scaled;
  gz_off = gz_scaled;

}

void getMotion6Scaled(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax_scaled = ax / accel_scale;
  ay_scaled = ay / accel_scale;
  az_scaled = az / accel_scale;
  gx_scaled = gx / gyro_scale;
  gy_scaled = gy / gyro_scale;
  gz_scaled = gz / gyro_scale;
}

void get_y_rotation(double x, double y, double z, double* y_rotation){
  *y_rotation = ((atan2(x, dist(y, z))) * 4068.0) / 71.0;
}

void get_x_rotation(double x, double y, double z, double* x_rotation){
  *x_rotation = ((atan2(y, dist(x, z))) * -4068.0) / 71.0;
}

double dist(double a, double b){
  return sqrt((a * a) + (b * b));
}

u_long lasttime;
void loop(void){
  readMpu6050(&mpu6050data);
  if(millis() - lasttime > 30){
    lasttime = millis();
    //Poll udp server
    int noBytes = Udp.parsePacket();
    if ( noBytes ) {
        Udp.read(packetBuffer,noBytes); 
        if(packetBuffer[0] == '1'){
          char tmpstr[7]; 
          strncpy(tmpstr, (char*)packetBuffer+4, 6);
          int rumbleDuration = atoi(tmpstr);
  
          strncpy(tmpstr, (char*)packetBuffer+11, 3);
          int rumblePower = atoi(tmpstr);

          rumble(rumbleDuration, rumblePower);
  
          Serial.printf("Received udp package! Rumble duration: %d ~ Rumble power : %d", rumbleDuration, rumblePower);
        }else if(packetBuffer[0] == '2'){ //Shock
          
        }

        Serial.println();
    } 

    if(rumbleActivated == true && millis() > rumbleDuration){
      rumbleActivated = false;
      analogWrite(15, 0);
    }
  
    readJoystick(&joystickdata);
  
    readSwitches(&switchdata);    
    
    //Send new data  
    joystickdata.button = 0;
    sendUdpMessage(&joystickdata, &mpu6050data, &switchdata); 
  }else{
    delay(6);
  }
}

void readMpu6050(struct MPU6050Data *mpu6050data){
    getMotion6Scaled();
    
    gx_scaled -= gx_off;
    gy_scaled -= gy_off;
    gz_scaled -= gz_off;


    double gx_delta = (gx_scaled * 0.01);
    double gy_delta = (gy_scaled * 0.01);
    double gz_delta = (gz_scaled * 0.01);

    double rotationx, rotationy;
    get_y_rotation(ax_scaled, ay_scaled, az_scaled, &rotationy);
    get_x_rotation(ax_scaled, ay_scaled, az_scaled, &rotationx);

    pitch = 0.98 * (pitch + gy_delta) + (0.02 * rotationy);
    roll = 0.98 * (roll + gx_delta) + (0.02 * rotationx);    

    mpu6050data->pitch = pitch*100;
    mpu6050data->roll = roll*100;
    mpu6050data->yaw = 0;
    mpu6050data->ax = ax_scaled;
    mpu6050data->ay = ay_scaled;
    mpu6050data->az = az_scaled;

}

void readJoystick(struct JoystickData *joystickdata){
  joystickdata->x = ads.readADC_SingleEnded(1, 0) - joystickoffset_x;
  delay(8);
  joystickdata->y = ads.readADC_SingleEnded(0, 0) - joystickoffset_y;
}

void readSwitches(struct SwitchData *switchdata){
  switchdata->magnetSwitch = !digitalRead(12);  
}

void rumble(int duration, int power){
  rumbleActivated = true;
  analogWrite(15, power);
  rumbleDuration = millis() + duration;
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
  sprintf(sendBuffer, "%s%06d|%06d|%06d|", sendBuffer, mpu6050data->yaw, mpu6050data->pitch, mpu6050data->roll);
  sprintf(sendBuffer, "%s%01d|%01d", sendBuffer, switchdata->backSwitch, switchdata->magnetSwitch);
   sprintf(sendBuffer, "%s%06d|%06d|%06d", sendBuffer, mpu6050data->ax, mpu6050data->ay, mpu6050data->az);
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

