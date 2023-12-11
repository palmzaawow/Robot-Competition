// Encoder stuffs
#include <ESP32Encoder.h>
ESP32Encoder enc0;
ESP32Encoder enc1;

#define ENC0_A  33
#define ENC0_B  32

#define ENC1_A  34
#define ENC1_B  35

int64_t lastencR = 0;// Store previous encoder value of Right motor
int64_t lastencL = 0;// Store previous encoder value of Left motor

// WiFi stuffs
#include <WiFi.h>
const char* ssid = "Robotclub_KMITL_E12";
const char* password =  "Kakorsetyor2022";

String allString ;
char c;
IPAddress local_IP(192, 168, 0, 123);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 254, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secDNS(8, 8, 4, 4);

WiFiServer wifiServer(80);

// motors
#define in1   13
#define in2   14
#define in3   25
#define in4   27
#define pw1   12
#define pw2   26

#define M1Channel 0
#define M2Channel 1

#define PWM_freq 100

// LEDs
#define IND1  16
#define IND2  4
#define IND3  2
#define IND4  15

// IR sensors
#define Lsen  39
#define Rsen  36
#define SENSOR_DETECT_SPSTATE 1200 // out cruve 
#define SENSOR_DETECT_STATE 400 // GPIO state when Dectect non-white area (A.K.A outside the road).
#define SENSOR_NORMAL_STATE 0 // GPIO state when the Robot car is still on the road.
#define CORRECTION_DELAY 5 // How long the motor will correct the position.
#define CORRECT_SPEED 55

uint8_t M1pw = 0;
uint8_t M2pw = 0;
uint8_t dir = 0;
uint8_t break_flag = 0;

void motorInit() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  ledcSetup(M1Channel, PWM_freq, 8);
  ledcSetup(M2Channel, PWM_freq, 8);
  ledcAttachPin(pw1, M1Channel);
  ledcAttachPin(pw2, M2Channel);
}

void GPIOsInit() {
  pinMode(pw1, OUTPUT);
  pinMode(pw2, OUTPUT);
  pinMode(IND1, OUTPUT);
  pinMode(IND2, OUTPUT);
  pinMode(IND3, OUTPUT);
  pinMode(IND4, OUTPUT);

  pinMode(Lsen, INPUT);
  pinMode(Rsen, INPUT);
}

void setup() {
  Serial.begin(115200);
  GPIOsInit();
  motorInit();

  // Initialization for Encoder
  enc0.attachHalfQuad(ENC0_A, ENC0_B);
  enc1.attachHalfQuad(ENC1_A, ENC1_B);

  enc0.setCount(4096);
  enc1.setCount(4096);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  digitalWrite(IND1, 1);

  wifiServer.begin();
}


void STP () {
  ledcWrite(M1Channel, 255);
  ledcWrite(M2Channel, 255);
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}

uint8_t main_fsm = 0;

void loop() {
  WiFiClient client = wifiServer.available();

  if (client) {
    //Serial.println("Connected");
    if (client.connected()) {
      if (client.available()) {// Check for incoming command
        STP();// Stop motor immediately
        Serial.println("Received new packet!");
        while (client.available()) {// Receive all command
          c = client.read();
          allString += c;
          //Debug
          if (c == '\n') {
            Serial.println(allString);
            lastencR = 0;
            lastencL = 0;
            break_flag = 0;
            main_fsm = 1;
            break;
          }
        }
      }

    }// while(client.connected())
  }// if (client)

  // Main state machine
  switch (main_fsm) {
    case 0:// Idle case
      //Serial.println("main_fsm : 0");
      dir = 0;
      M1pw = 0;
      M2pw = 0;
      break;

    case 1:// Command parser.
      Serial.println("main_fsm : 1");
      M1pw = ((allString[4] - '0') * 100) + ((allString[5] - '0') * 10) + (allString[6] - '0') ;
      M2pw = ((allString[9] - '0') * 100) + ((allString[10] - '0') * 10) + (allString[11] - '0') ;

      lastencR = (((allString[15] - '0') * 10) + (allString[16] - '0'));
      lastencL = (((allString[17] - '0') * 10) + (allString[18] - '0'));
      
      // Stop motor
      if (allString[1] == '0') {
        STP();
        main_fsm = 0;
        break;
      }

      //Debug
      Serial.print("M1:");
      Serial.print(M1pw);
      Serial.print("M2:");
      Serial.println(M2pw);

      Serial.print("R step:");
      Serial.print(lastencR);
      Serial.print("L step:");
      Serial.println(lastencL);

      // LED stuffs
      if (allString[12] == '1') {
        digitalWrite(IND2, 1);
      }
      else {
        digitalWrite(IND2, 0);
      }

      if (allString[13] == '1') {
        digitalWrite(IND3, 1);
      }
      else {
        digitalWrite(IND3, 0);
      }

      if (allString[14] == '1') {
        digitalWrite(IND4, 1);
      }
      else {
        digitalWrite(IND4 , 0);
      }
      // End LED stuffs

      // parse Right motor direction
      if (allString[3] == 'c') {
        dir |= (1 << 7);
      } else if (allString[3] == 't') {
        dir &= ~(1 << 7);
      }

      // parse Left motor direction
      if (allString[8] == 'c') {
        dir |= (1 << 6);
      } else if (allString[8] == 't') {
        dir &= ~(1 << 6);
      }

      main_fsm = 2;

      // Enter Special mode
      if (allString[0] == '1') {
        main_fsm = 4;
        break;
      }

      allString = "";// Clear command buffer

      break;

    case 2:// Motor control
      Serial.println("main_fsm : 2");
      // Get latest Encoder counter value
      ledcWrite(M1Channel, M1pw);
      ledcWrite(M2Channel, M2pw);

      // Check Bit 7 and 6 for motor direction control
      switch (dir & 0xC0) {
        case 0x80:// R : Forward, L : Backward
          lastencR *= 1115;
          lastencL *= 1115;
          lastencR = enc1.getCount() + lastencR;
          lastencL = enc0.getCount() - lastencL;

          // Right
          digitalWrite(in1, 0);
          digitalWrite(in2, 1);
          // Left
          digitalWrite(in3, 1);
          digitalWrite(in4, 0);
          break;

        case 0x40:// R : Backward, L : Forward
          lastencR *= 1115;
          lastencL *= 1115;
          lastencR = enc1.getCount() - lastencR;
          lastencL = enc0.getCount() + lastencL;
          // Right
          digitalWrite(in1, 1);
          digitalWrite(in2, 0);
          // Left
          digitalWrite(in3, 0);
          digitalWrite(in4, 1);
          break;

        case 0xC0:// R : Forward, L : Forward
          // Full revolution to steps
          lastencR *= 512;
          lastencL *= 512;
          lastencR = enc1.getCount() + lastencR;
          lastencL = enc0.getCount() + lastencL;
          // Right
          digitalWrite(in1, 0);
          digitalWrite(in2, 1);
          // Left
          digitalWrite(in3, 0);
          digitalWrite(in4, 1);
          break;

        case 0x00:// R : Backward, L : Backward
          lastencR *= 512;
          lastencL *= 512;
          lastencR = enc1.getCount() - lastencR;
          lastencL = enc0.getCount() - lastencL;
          // Right
          digitalWrite(in1, 1);
          digitalWrite(in2, 0);
          // Left
          digitalWrite(in3, 1);
          digitalWrite(in4, 0);
          break;
      }//  switch (dir & 0xC0)

      Serial.print("Last ENC R :");
      Serial.print(lastencR);
      Serial.print(" Last ENC L :");
      Serial.println(lastencL);
      main_fsm = 3;

    // WARNING : Code Flow through from fsm state 2 to state 3
    // WARNING : Code Flow through from fsm state 2 to state 3
    // WARNING : Code Flow through from fsm state 2 to state 3
    // WARNING : Code Flow through from fsm state 2 to state 3
    // WARNING : Code Flow through from fsm state 2 to state 3

    case 3:// Encoder stuffs
      Serial.println("main_fsm : 3");
      //delay(1);// <1000Hz PID scan rate.
      // Check Bit 7 and 6 for motor direction control
      switch (dir & 0xC0) {
        case 0x80:// R : Forward, L : Backward
          Serial.println("Turn Left");
          // Right
          if (enc1.getCount() >= lastencR) {
            digitalWrite(in1, 1);
            digitalWrite(in2, 0);
            ledcWrite(M1Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in1, 0);
            break_flag++;
          }

          // Left
          if (enc0.getCount() <= lastencL) {
            digitalWrite(in4, 1);
            digitalWrite(in3, 0);
            ledcWrite(M2Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in4, 0);
            break_flag++;
          }

          if (break_flag >= 2) {
            STP();
            break_flag = 0;
            main_fsm = 0;
            //                enc0.setCount(4096);
            //                enc1.setCount(4096);
            break;
          }

          break;

        case 0x40:// R : Backward, L : Forward
          Serial.println("Turn Right");
          // Right
          if (enc1.getCount() <= lastencR) {
            digitalWrite(in2, 1);
            digitalWrite(in1, 0);
            ledcWrite(M1Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in2, 0);
            break_flag++;
          }

          // Left
          if (enc0.getCount() >= lastencL) {
            digitalWrite(in3, 1);
            digitalWrite(in4, 0);
            ledcWrite(M2Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in3, 0);
            break_flag++;
          }

          if (break_flag >= 2) {
            STP();
            break_flag = 0;
            main_fsm = 0;
            //                enc0.setCount(4096);
            //                enc1.setCount(4096);
            break;
          }

          break;

        case 0xC0:// R : Forward, L : Forward
          Serial.println("Forward");
          // Stop right hand motor to make the robot correct itself to right hand side
          if (analogRead(Lsen) > SENSOR_DETECT_STATE) {
            ledcWrite(M1Channel, CORRECT_SPEED);
            ledcWrite(M2Channel, CORRECT_SPEED);
            digitalWrite(in1, 0);
            digitalWrite(in2, 0);
            delay(CORRECTION_DELAY);
            digitalWrite(in1, 0);
            digitalWrite(in2, 1);
            ledcWrite(M1Channel, M1pw);
            ledcWrite(M2Channel, M2pw);
          }

          // Stop left hand motor to make the robot correct itself to left hand side
          if (analogRead(Rsen) > SENSOR_DETECT_STATE) {
            ledcWrite(M1Channel, CORRECT_SPEED);
            ledcWrite(M2Channel, CORRECT_SPEED);
            digitalWrite(in3, 0);
            digitalWrite(in4, 0);
            delay(CORRECTION_DELAY);
            digitalWrite(in3, 0);
            digitalWrite(in4, 1);
            ledcWrite(M1Channel, M1pw);
            ledcWrite(M2Channel, M2pw);
          }


          // Right
          if (enc1.getCount() >= lastencR) {
            digitalWrite(in1, 1);
            digitalWrite(in2, 0);
            ledcWrite(M1Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in1, 0);
            break_flag++;
            Serial.print("Break flag:");
            Serial.print(break_flag);
          }

          // Left
          if (enc0.getCount() >= lastencL) {
            digitalWrite(in3, 1);
            digitalWrite(in4, 0);
            ledcWrite(M2Channel, 255);
            delayMicroseconds(200);
            digitalWrite(in3, 0);
            break_flag++;
            Serial.print("Break flag:");
            Serial.print(break_flag);
          }

          if (break_flag >= 2) {
            STP();
            break_flag = 0;
            main_fsm = 0;
            //                enc0.setCount(4096);
            //                enc1.setCount(4096);
            break;
          }

          Serial.println("main_fsm : 3 end");

          break;

        case 0x00:// R : Backward, L : Backward
          Serial.println("Backward");

          // Right
          if (enc1.getCount() <= lastencR) {
            digitalWrite(in2, 1);
            digitalWrite(in1, 0);
            ledcWrite(M1Channel, 255);
            digitalWrite(in2, 0);
            break_flag++;
          }

          // Left
          if (enc0.getCount() <= lastencL) {
            digitalWrite(in4, 1);
            digitalWrite(in3, 0);
            ledcWrite(M2Channel, 255);
            digitalWrite(in4, 0);
            break_flag++;
          }

          if (break_flag >= 2) {
            STP();
            break_flag = 0;
            main_fsm = 0;
            //                enc0.setCount(4096);
            //                enc1.setCount(4096);
            break;
          }

          break;

      }// switch(dir & 0xC0)

      break;

    case 4:// Curving
      ledcWrite(M1Channel, M1pw);
      ledcWrite(M2Channel, M2pw);

      // Right
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);
      // Left
      digitalWrite(in3, 0);
      digitalWrite(in4, 1);

      if (analogRead(Lsen) > SENSOR_DETECT_SPSTATE) {
        ledcWrite(M1Channel, CORRECT_SPEED);
        ledcWrite(M2Channel, CORRECT_SPEED);
        digitalWrite(in1, 0);
        digitalWrite(in2, 0);
        delay(CORRECTION_DELAY);
        digitalWrite(in1, 0);
        digitalWrite(in2, 1);
        ledcWrite(M1Channel, M1pw);
        ledcWrite(M2Channel, M2pw);
      }

      // Stop left hand motor to make the robot correct itself to left hand side
      if (analogRead(Rsen) > SENSOR_DETECT_SPSTATE) {
        ledcWrite(M1Channel, CORRECT_SPEED);
        ledcWrite(M2Channel, CORRECT_SPEED);
        digitalWrite(in3, 0);
        digitalWrite(in4, 0);
        delay(CORRECTION_DELAY);
        digitalWrite(in3, 0);
        digitalWrite(in4, 1);
        ledcWrite(M1Channel, M1pw);
        ledcWrite(M2Channel, M2pw);
      }

      break;
  }// switch(main_fsm)

}// void loop
