
#include "CRU.h"
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

// 16 17 5 18 19 21


const char* ssid = "Robotclub_KMITL_E12";
const char* password = "Kakorsetyor2022";

uint32_t timeOutCounter = 0;

const int freq = 5000;
const int lockChannel = 0;
const int feedChannel = 1;
const int resolution = 8;

typedef struct struct_message {
  uint8_t feedMotor;
  uint8_t lockMotor;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  timeOutCounter = 0;
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
  

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  WiFi.mode(WIFI_STA);
 // Serial.begin(115200); //V3
  
  WiFi.setSleep(false);
  Serial.begin(115200);
  /*WiFi.begin(ssid, password);  // V3
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
*/
  
  if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initializing ESP-NOW");
    return;
  }
  //CRU.init();

ledcWrite(feedChannel, 4095);
ledcWrite(lockChannel, 3600);

  esp_now_register_recv_cb(OnDataRecv);

  
}
 
void loop() {
  if(myData.feedMotor == 1){
  /*  CRU.writeMotor(2,4095,0x01);
    CRU.writeInd(0x04);
    CRU.update();
  */
  digitalWrite(mot1,1);
  digitalWrite(mot2,0);
  //ledcWrite(feedChannel, 4095);
  // feed in
  
  
  }
  else if(myData.feedMotor == 2){
    /*CRU.writeMotor(2,4095,0x02);
    CRU.writeInd(0x08);
    CRU.update();
    */
    digitalWrite(mot1,0);
    digitalWrite(mot2,1);
   // ledcWrite(feedChannel, 4095);
   // feed out
  }
  else{
    
    /*
    CRU.writeMotor(2,4095,0x00);
    CRU.writeInd(0x01);
    CRU.update();
    */
     digitalWrite(mot1,0);
     digitalWrite(mot2,0);
    //ledcWrite(feedChannel, 4095);
   //stop feed
  }

  if(myData.lockMotor == 2){
    /*CRU.writeMotor(3,3600,0x01);
    CRU.update();
    */
     digitalWrite(mot3,1);
     digitalWrite(mot4,0);
   // ledcWrite(lockChannel, 3600);
    // lock in 
  }
  else if (myData.lockMotor == 1){
  /*  CRU.writeMotor(3,3600,0x02);
    CRU.update();
   */
   digitalWrite(mot3,0);
   digitalWrite(mot4,1);
  // ledcWrite(lockChannel, 3600);
    // lock out 
  }
  else{
    /*
    CRU.writeMotor(3,3600,0);
    CRU.update();
    */
    digitalWrite(mot3,0);
    digitalWrite(mot4,0);
   // ledcWrite(lockChannel, 3600);
    // stop lock
  }

  
  timeOutCounter++;
  if(timeOutCounter > 250){
    /*
    for(uint8_t i = 0;i<4;i++){
      CRU.writeMotor(i,0,0);
      //stop motor
    }
    CRU.writeInd(0x03);
    CRU.update();
    */
     digitalWrite(mot1,0);
     digitalWrite(mot2,0);
     digitalWrite(mot3,0);
     digitalWrite(mot4,0);
    ledcWrite(feedChannel, 4095);
    ledcWrite(lockChannel, 4095);
      //stop moter
  }
  else{
    
 //   CRU.update();
  }
  delay(1);



}
