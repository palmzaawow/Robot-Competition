#include "stdint.h"
#include "E12_Function.h"
uint8_t broadcastAddress[] = { 0x40, 0x22, 0xD8, 0x57, 0xEA, 0x94 }; // 40:22:D8:57:EA:94    0x30, 0xC6, 0xF7, 0x00, 0x4D, 0x84 
uint8_t returnAddress[] = { 0x40, 0x22, 0xD8, 0x4F, 0x43, 0xCC}; // 40:22:D8:4F:43:CC newbbb 0xC0, 0x49, 0xEF, 0x69, 0x8D, 0x28 
 
#define delayTime 500

#define m1x 1.0
#define m1y 1.0
#define m1z 1.0
#define m2x 1.0
#define m2y 1.0
#define m2z 1.0
#define m3x 1.0
#define m3y 1.0
#define m3z 1.0
#define m4x 1.0
#define m4y 1.0
#define m4z 1.0

#define maxSpeed 4200
#define threshold 350
#define maxRollerSpeed 5200
uint8_t packet[1024];
uint64_t timeOutCounter = 0;
volatile uint64_t generalCounter = 0;
int16_t M1 = 0, M2 = 0, M3 = 0;
uint8_t indConfig = 0b11111111;

uint8_t RXPacket[1024];
uint8_t TXPacket[1024];
uint8_t RXPacket2[1024];
uint8_t TXPacket2[1024];

uint32_t prevMillis = 0;
uint8_t commuFault = 0;

uint8_t controlMode = 0;

uint8_t cmdCounter = 0;

motorParameter_t motorParameterDefault = { 0, 0, 0, 0, 0, 0, 24000, 0, 0, 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 0, 0 };
speedControl_t speedControlDefault = { 0, 0, 0, 0 };
positionControl_t positionControlDefault = { 0, 0, 0, 0, 0 };
motor motorDefault = { motorParameterDefault,
                       speedControlDefault,
                       positionControlDefault,
                       speedControlCMD };
uint8_t displayEnable = 0;
float speedCoefficient = 0.75;
uint8_t speedFlag[4] = { 0, 0, 0, 0 };
float calculatedSpeed[4] = { 0, 0, 0, 0 };
float mSpeed[4] = { 0, 0, 0, 0 };
uint32_t counter5 = 0;

motor MOTOR[4] = { motorDefault, motorDefault, motorDefault, motorDefault };
uint32_t firstMillis = 0;
QuadMotorDriver Q1(MOTOR);
QuadMotorDriver Q2(MOTOR);
uint32_t prevMillisElevation = 0;
uint32_t prevMillisLoad = 0;

uint8_t stateRT = 1, stateRD = 1, stateRL = 1, stateRR = 1;
uint8_t stateLT = 1, stateLD = 1, stateLL = 1, stateLR = 1;

uint8_t stateCom1 = 1, stateCom2 = 1, stateCom3 = 1, stateCom4 = 1, stateCom5 = 1;
uint8_t laststateRT = 1, laststateRD = 1, laststateRL = 1, laststateRR = 1;
uint8_t laststateLT = 1, laststateLD = 1, laststateLL = 1, laststateLR = 1;

byte incomingByte;
double sensorData[6] = { 0, 0, 0, 0, 0, 0 };
int ind1 = LOW;

float upperRollerSpeed = 0;
float lowerRollerSpeed = 0;
volatile uint8_t robotStarted = 1;
uint16_t maxMotorSpeed = 0;

int32_t prevEncL = 0;
int32_t prevEncR = 0;

int32_t elevationCounter = 0;
int32_t prevElevationCounter = 0;
#define elevationInterval 100
uint32_t prevElevationMillis = 0;
uint8_t elevationDir = 0;
uint8_t elevationRunning = 0;

int32_t collectingArmCounter = 0;
int32_t prevCollectingArmCounter = 0;
#define collectingArmInterval 50
uint32_t prevCollectingArmMillis = 0;
uint8_t collectingArmDir = 0;
uint8_t collectingArmRunning = 0;


const char* ssid = "Robotclub_KMITL_E12";
const char* password = "Kakorsetyor2022";


typedef struct {
  uint32_t started;
  int32_t id;
  int32_t encl, encr;
  int32_t x, y, z, w;
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  uint8_t com1, com2, com3, com4, com5;
} struct_message;

typedef struct {
  uint32_t acknowledge;
} returnStruct_t;
returnStruct_t returnPacket;

typedef struct sub_message {
  uint8_t feedMotor;
  uint8_t lockMotor;
} sub_message;

sub_message subData;

struct_message myData;

hw_timer_t *timer1 = NULL;

float RPM[4] = { 0, 0, 0, 0 };
int16_t xComponent = 0, yComponent = 0, zComponent = 0, wComponent = 0;

void calculateElevationSpeed() {
  if (elevationCounter > prevElevationCounter && !elevationRunning) {
    prevElevationMillis = millis();
    elevationDir = 1;
    Q2.setPWM(3, 4200, elevationDir);
    elevationRunning = 1;
  } else if (elevationCounter < prevElevationCounter && !elevationRunning) {
    prevElevationMillis = millis();
    elevationDir = 2;
    Q2.setPWM(3, 4200, elevationDir);
    elevationRunning = 1;
  }

  if (millis() - prevElevationMillis > elevationInterval && elevationRunning) {

    if (elevationDir == 1) {
      prevElevationCounter++;
    } else if (elevationDir == 2) {
      prevElevationCounter--;
    } else {
      prevElevationCounter = elevationCounter;
    }

    if (prevElevationCounter == elevationCounter) {
      Q2.setPWM(3, 4200, 0);
    }
    elevationRunning = 0;
  }
}

void calculateSubSystem() {
  if (!stateRT) {
    subData.lockMotor = 1;
  } else if (!stateRD) {
    subData.lockMotor = 2;
  } else {
    subData.lockMotor = 0;
  }

  if (!stateRR) {
    subData.feedMotor = 1;
  } else if (!stateRL) {
    subData.feedMotor = 2;
  } else {
    subData.feedMotor = 0;
  }
  esp_now_send(broadcastAddress, (uint8_t *)&subData, sizeof(sub_message));
}

void calculateRollerSpeed(struct_message myData) {
  /*
  if (!stateLD) {
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
  } else if (!stateLR) {
    upperRollerSpeed = -2000;
    lowerRollerSpeed = -2000;
  } else if (!stateLT) {
    upperRollerSpeed = -4000;
    lowerRollerSpeed = -4000;
  } else if (!stateLL) {
    upperRollerSpeed = -5200;
    lowerRollerSpeed = -5200;
  }*/

    if (myData.id == 2) {

    
    upperRollerSpeed = -1 * myData.com1 * 0.01 * 5200;
    lowerRollerSpeed = -1 * myData.com2 * 0.01 * 5200;
    
   // Serial.print(upperRollerSpeed);
    //Serial.print(" "); 
   // Serial.println(lowerRollerSpeed);
    /*
   upperRollerSpeed = 0.8 * maxRollerSpeed;
    lowerRollerSpeed = -0.7 * maxRollerSpeed;
    */

  }
  else{
      if (!stateLD) {
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
  }
  
    if (!stateCom4){
      
    if (!stateLR) {
    upperRollerSpeed = -2000;
    lowerRollerSpeed = -2000;
  } else if (!stateLT) {
    upperRollerSpeed = -3000;
    lowerRollerSpeed = -3000;
  } else if (!stateLL) {
    upperRollerSpeed = -4000;
    lowerRollerSpeed = -4000;
  }
  
    }
    else{
      
      if (!stateLR) {
    upperRollerSpeed = -2000;
    lowerRollerSpeed = -2500;
  } else if (!stateLT) {
    upperRollerSpeed = -3000;
    lowerRollerSpeed = -3500;
  } else if (!stateLL) {
    upperRollerSpeed = -5200;
    lowerRollerSpeed = -5200;
  }
  
    }
  
  //rollerSpeed

  if (myData.encl != prevEncL) {
    //collectingArmCounter += (myData.encl - prevEncL);
    elevationCounter += (myData.encl - prevEncL);
    prevEncL = myData.encl;
  }

    if (myData.encr != prevEncR) {
   // elevationCounter += (myData.encr - prevEncR);
     collectingArmCounter += (myData.encr - prevEncR);
    prevEncR = myData.encr;
  }
}

  if (upperRollerSpeed < -5200) {
    upperRollerSpeed = -5200;
  } else if (upperRollerSpeed > -1000) {
    upperRollerSpeed = 0;
  }

  if (lowerRollerSpeed < -5200) {
    lowerRollerSpeed = -5200;
  } else if (lowerRollerSpeed > -1000) {
    lowerRollerSpeed = 0;
  }


}

void calculateLiftSpeed(struct_message myData) {
  if (myData.id == 1){
  if (!stateCom1) {
    Q2.setPWM(4, 2600, 0x01);
    //Serial.println("A");
  } else if (!stateCom2) {
    Q2.setPWM(4, 2600, 0x02);
  } else {
    Q2.setPWM(4, 2600, 0);
  }
  }
}

void calculateCollectingArm() {
  if (collectingArmCounter > prevCollectingArmCounter && !collectingArmRunning) {
    prevCollectingArmMillis = millis();
    collectingArmDir = 1;
    Q1.setPWM(4, 4200, collectingArmDir);
    collectingArmRunning = 1;
  } else if (collectingArmCounter < prevCollectingArmCounter && !collectingArmRunning) {
    prevCollectingArmMillis = millis();
    collectingArmDir = 2;
    Q1.setPWM(4, 4200, collectingArmDir);
    collectingArmRunning = 1;
  }

  if (millis() - prevCollectingArmMillis > collectingArmInterval && collectingArmRunning) {

    if (collectingArmDir == 1) {
      prevCollectingArmCounter++;
    } else if (collectingArmDir == 2) {
      prevCollectingArmCounter--;
    } else {
      prevCollectingArmCounter = collectingArmCounter;
    }

    if (prevCollectingArmCounter == collectingArmCounter) {
      Q1.setPWM(4, 4200, 0);
    }
    collectingArmRunning = 0;
  }
}

void calculateSpeed() {

  if (myData.w > 2047 + threshold || myData.w < 2047 - threshold) {
    yComponent = map(myData.w, 0, 4095, -2048, 2047);
  } else {
    yComponent = 0;
  }


  if (myData.z > 2047 + threshold || myData.z < 2047 - threshold) {
    xComponent = map(myData.z, 0, 4095, 2047, -2048);
  } else {
    xComponent = 0;
  }

  if (myData.x > 2047 + threshold || myData.x < 2047 - threshold) {
    zComponent = map(myData.x, 0, 4095, -2048, 2047);
  } else {
    zComponent = 0;
  }

  if (myData.y > 2047 + threshold || myData.y < 2047 - threshold) {
    wComponent = map(myData.y, 0, 4095, -2048, 2047);
  } else {
    wComponent = 0;
  }

  if (!stateCom3) {
    controlMode = 1;
  } else {
    controlMode = 0;
  }
  //Serial.println(controlMode);


  switch (controlMode) {
    case 0:
      M1 = (int16_t)(m1x * xComponent - m1z * zComponent * 0.75) / 2048.00 * 4200.00;
      M2 = (int16_t)(-m2y * yComponent * 0.88603 + m2x * xComponent * 0.88603 + m2z * zComponent * 0.75) / 2048.00 * 4200.00;
      M3 = (int16_t)(m3y * yComponent * 0.88603 + m3x * xComponent * 0.88603 + m3z * zComponent * 0.75) / 2048.00 * 4200.00;
      break;
    case 1:
      /*M1 = 0;
      M2 = map(zComponent, -2048, 2047, -3200, 3200);
      M3 = map(wComponent, -2048, 2047, -4200, 4200);*/
      M1 = (int16_t)(m1x * xComponent - m1z * zComponent * 1) / 2048.00 * 4200.00;
      M2 = (int16_t)(-m2y * yComponent * 0.88603 + m2x * xComponent * 0.88603 + m2z * zComponent * 1) / 2048.00 * 4200.00;
      M3 = (int16_t)(m3y * yComponent * 0.88603 + m3x * xComponent * 0.88603 + m3z * zComponent * 1) / 2048.00 * 4200.00;
      break;
  }
  maxMotorSpeed = abs(M1);
  maxMotorSpeed = max(abs(M1), abs(M2));
  maxMotorSpeed = max(abs(M2), abs(M3));

  if (maxMotorSpeed < 4200) {
    maxMotorSpeed = 4200;
  }

  M1 = (float)M1 / maxMotorSpeed * 4200.00 * speedCoefficient;
  M2 = (float)M2 / maxMotorSpeed * 4200.00 * speedCoefficient;
  M3 = (float)M3 / maxMotorSpeed * 4200.00 * speedCoefficient;

  if (M1 > 4200) {
    Q1.setPWM(1, 4200, 0x01);
  } else if (M1 > 0) {
    Q1.setPWM(1, M1, 0x01);
  } else if (M1 == 0) {
    Q1.setPWM(1, 3600, 0);
  } else if (M1 > -4200) {
    Q1.setPWM(1, -M1, 0x02);
  } else {
    Q1.setPWM(1, 4200, 0x02);
  }

  if (M2 > 4200) {
    Q1.setPWM(2, 4200, 0x02);
  } else if (M2 > 0) {
    Q1.setPWM(2, M2, 0x02);
  } else if (M2 == 0) {
    Q1.setPWM(2, 3600, 0);
  } else if (M2 > -4200) {
    Q1.setPWM(2, -M2, 0x01);
  } else {
    Q1.setPWM(2, 4200, 0x01);
  }

  if (M3 > 4200) {
    Q1.setPWM(3, 4200, 0x02);
  } else if (M3 > 0) {
    Q1.setPWM(3, M3, 0x02);
  } else if (M3 == 0) {
    Q1.setPWM(3, 3600, 0);
  } else if (M3 > -4200) {
    Q1.setPWM(3, -M3, 0x01);
  } else {
    Q1.setPWM(3, 4200, 0x01);
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  firstMillis = millis();
  memcpy(&myData, incomingData, sizeof(myData));
  timeOutCounter = 0;
if (myData.id == 1){
  stateRT = myData.rt;
  stateRD = myData.rd;
  stateRL = myData.rl;
  stateRR = myData.rr;
  stateLT = myData.lt;
  stateLD = myData.ld;
  stateLL = myData.ll;
  stateLR = myData.lr;
  stateCom1 = myData.com1;
  stateCom2 = myData.com2;
  stateCom3 = myData.com3;
  stateCom4 = myData.com4;
  stateCom5 = myData.com5;
  if (robotStarted) {
    prevEncL = myData.encl;
    prevEncR = myData.encr;
    robotStarted = 0;
  }
 
  if (myData.started) {
    returnPacket.acknowledge = 1;
     esp_now_send(returnAddress, (uint8_t *)&returnPacket, sizeof(returnPacket));
    prevEncL = 0;
    prevEncR = 0;
    return;
  }
  calculateSpeed();
  //calculateLiftSpeed(myData);
  //calculateElevationSpeed();
  calculateCollectingArm();

}

     calculateRollerSpeed(myData);
 

 /* Serial.println(stateCom1);
  Serial.println(stateCom2);
  Serial.println(stateCom3);
  Serial.println(stateCom4);
  Serial.println(stateCom5);
  Serial.println("");
  */
  /*
    Serial.print(myData.rt);
    Serial.print(" ");
    Serial.print (myData.rd);
    Serial.print (" ");
    Serial.print (myData.rl);
    Serial.print(" ");
    Serial.print (myData.rr);
    Serial.print(" ");
    Serial.print (myData.lt);
    Serial.print(" ");
    Serial.print (myData.ld);
    Serial.print(" ");
    Serial.print (myData.ll);
    Serial.print(" ");
    Serial.print (myData.lr);
    Serial.print(" ");
    Serial.print (myData.encl);
    Serial.print(" ");
    Serial.print (myData.encr);
    Serial.print(" ");
    Serial.print (myData.x);
    Serial.print(" ");
    Serial.print (myData.y);
    Serial.print(" ");
    Serial.print (myData.z);
    Serial.print(" ");
    Serial.print (myData.w);
    Serial.println(" ");*/
  
}
/*
void IRAM_ATTR onTimer() {
  updateSpeed();
  for (uint8_t j = 0; j < 4; j++) {
    Q1.setCMD(j + 1, speedControlCMD);
  }
  Q1.getPacket(TXPacket);
  sendData();
  generalCounter++;
}*/

void setPIDParameters() {
  Q2.setSpeedPID(1, 1.8, 2, 0);
  Q2.setSpeedPID(2, 1.8, 2, 0);
}

void setup() {
  setPIDParameters();
  E12BusSetup(16000000);
  for (uint16_t i = 0; i < 256; i++) {
    packet[i] = i + 1;
  }


  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
   Serial.begin(115200);

 /*  WiFi.setSleep(false);
    Serial.begin(115200);
  WiFi.begin(ssid, password);  // V3
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  */
  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_send_cb(OnDataSent);
 esp_now_peer_info_t peerInfo = {};
/*
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
*/
  memcpy(peerInfo.peer_addr, returnAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  for (uint8_t i = 0; i < 4; i++) {
    Q1.setPWM(i + 1, 0, 0);
  }
  upperRollerSpeed = 0;
  lowerRollerSpeed = 0;
  //Q2.setSpeed(1, upperRollerSpeed);
  //Q2.setSpeed(2, lowerRollerSpeed);
  //Q2.setPWM(3, 0, 0);
  //Q2.setPWM(4, 0, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void loop() {
 // calculateSubSystem();

  timeOutCounter++;
  if (timeOutCounter > 200) {
    for (uint8_t i = 0; i < 4; i++) {
      Q1.setPWM(i + 1, 0, 0);
    }
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
   /* Q2.setSpeed(1, upperRollerSpeed);
    Q2.setSpeed(2, lowerRollerSpeed);
    Q2.setPWM(3, 0, 0);
    Q2.setPWM(4, 0, 0);
    */
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  } else {
    indConfig |= (1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  //serialReceive();

  if (cmdCounter > 0) {
    for (uint8_t j = 0; j < 4; j++) {
      Q1.setCMD(j + 1, directControlCMD);
    }
/*
    Q2.setSpeed(1, upperRollerSpeed);
    Q2.setSpeed(2, lowerRollerSpeed);
    Q2.setCMD(1, speedControlCMD);
    Q2.setCMD(2, speedControlCMD);
    Q2.setCMD(3, directControlCMD);
    Q2.setCMD(4, directControlCMD);
*/
    Q1.getPacket(TXPacket);
 //   Q2.getPacket(TXPacket2);
    sendData();
    cmdCounter--;
  } else {
    for (uint8_t j = 0; j < 4; j++) {
      Q1.setCMD(j + 1, motorParameterCMD);
    }
    Q1.getPacket(TXPacket);

    for (uint8_t j = 0; j < 4; j++) {
      Q2.setCMD(j + 1, motorParameterCMD);
    }
    Q2.getPacket(TXPacket2);

    sendData();
    cmdCounter = 10;
  }



  indConfig &= ~(1 << 0);
  peripheralWrite(0, 0xFF, 0xFC, indConfig);
  delay(1);
  indConfig |= (1 << 0);
  peripheralWrite(0, 0xFF, 0xFF, indConfig);
}
