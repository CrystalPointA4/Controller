extern "C" {
  #include "user_interface.h"
}
#include <Wire.h>
#include <ESP8266WiFi.h>

String WifiSSIDPrefix = "CrystalPoint";
char WifiPassword[10];
char WifiSSID[17];

void setup(void){
  Serial.begin(115200);

  //LED
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  
  delay(100);
  setupWifi();
}

void loop(void){

  //main loop

  delay(100);
}

void setupWifi(void){
  WiFi.disconnect(true);
  WiFi.mode(...);

  generateWiFiPassword();
}

void generateWiFiPassword()
{
  String WifiStringPassword;
  for(char i=0; i < 17; i++){
      String s = String(WifiSSID[i], HEX);
      WifiStringPassword = s + WifiStringPassword;
  }
  for (int i=0; i < 10; i++)
    WifiPassword[i] = WifiStringPassword.charAt(i);
  WifiPassword[9] = '\0';
  Serial.print("Wifi password: ");  
  Serial.println(WifiPassword);
}
