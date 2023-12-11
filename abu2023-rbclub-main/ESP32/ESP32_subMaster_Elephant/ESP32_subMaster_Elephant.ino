
#include <esp_now.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


#define pwm1 16
#define mot1 17
#define mot2 5
#define pwm2 18
#define mot3 19
#define mot4 21

const char* ssid = "Robotclub_KMITL_E12";
const char* password = "Kakorsetyor2022";

const int freq = 5000;
const int lockChannel = 0;
const int feedChannel = 1;
const int resolution = 8;

typedef struct struct_message {
  uint8_t gripper;
  uint8_t feed;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  if(myData.gripper == 0){
    digitalWrite(mot1,0);
    digitalWrite(mot2,1);
  }
  else if(myData.gripper == 1){
    digitalWrite(mot1,1);
    digitalWrite(mot2,0);
  }
  else{
    digitalWrite(mot1,0);
    digitalWrite(mot2,0);
  }
  

  if(myData.feed == 0){
    ledcWrite(lockChannel, 160);
    digitalWrite(mot3,1);
    digitalWrite(mot4,0);
  }
  else if(myData.feed == 1){
    ledcWrite(lockChannel, 218);
    digitalWrite(mot3,0);
    digitalWrite(mot4,1);
  }
  else{
    ledcWrite(lockChannel, 255);
    digitalWrite(mot3,0);
    digitalWrite(mot4,0);
  }  
}
 
void setup() {

  pinMode(pwm1,OUTPUT);
  pinMode(mot1,OUTPUT);
  pinMode(mot2,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(mot3,OUTPUT);
  pinMode(mot4,OUTPUT);

  ledcSetup(lockChannel, freq, resolution);
  ledcSetup(feedChannel, freq, resolution);
  
  ledcAttachPin(pwm2, lockChannel);
  ledcAttachPin(pwm1, feedChannel);
  
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
   /*   
     WiFi.begin(ssid, password);   //dfsdfsdfsd
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  */
  if (esp_now_init() != ESP_OK) {
    return;
  }
 

  esp_now_register_recv_cb(OnDataRecv);

  ledcWrite(feedChannel, 150);
  
}
 
void loop() {
  delay(1);
}
