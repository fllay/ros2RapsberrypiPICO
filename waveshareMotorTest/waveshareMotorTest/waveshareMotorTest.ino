

#include "hardware/uart.h"

#include <Scheduler.h>






UART Serial2(8, 9);

int leftRPM;
int rightRPM;

int currentLeftRPM;
int currentRightRPM;


bool lock1 = false;
bool lock2 = false;

uint8_t accel = 0x05;

uint8_t checkSpeed[10] = { 0x01, 0x74, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte* data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

int setM1Speed(int speed) {
  uint8_t Speed[10] = { 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, accel, 0x00, 0x00, 0x00 };
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  //Serial.println("Set M1 Speed");
  if (lock1 == false) {
    lock1 = true;
    Serial1.write(Speed, sizeof(Speed));
    while (Serial1.available() < 10) {

    }  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = Serial1.read();
    number = (int16_t)(buffer[2] << 8) + buffer[3];

    lock1 = false;
  }

  return number;
  //Serial.println("Set M1 Speed done");
}


int setM2Speed(int speed) {
  uint8_t Speed[10] = { 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, accel, 0x00, 0x00, 0x00 };
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  //Serial.println("Set M2 Speed");
  if (lock1 == false) {
    lock1 = true;
    Serial2.write(Speed, sizeof(Speed));
    while (Serial2.available() < 10) {

    }  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = Serial2.read();
    number = (int16_t)(buffer[2] << 8) + buffer[3];

    lock1 = false;
  }
  return number;
  //Serial.println("Set M2 Speed done");
}



int getM1Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock1 == false) {
    lock1 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    Serial1.write(checkSpeed, sizeof(checkSpeed));
    while (Serial1.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = Serial1.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock1 = false;
  }
  return number;
}

int getM2Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock2 == false) {
    lock2 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    Serial2.write(checkSpeed, sizeof(checkSpeed));
    while (Serial2.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = Serial2.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock2 = false;
  }
  return number;
}


void m1ctl() {  //Right motor
  currentRightRPM =  setM1Speed(-10 * rightRPM)/-10;
  delay(10);
}


void m2ctl() {  // Left motor
  currentLeftRPM = setM2Speed(10*leftRPM)/10;
  delay(10);
}


void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  rightRPM = 100;
  leftRPM = 100;
   delay(5000);
   Serial.println("Positive");
  Serial.println(currentLeftRPM);
  Serial.println(currentRightRPM);
 
  digitalWrite(LED_BUILTIN, HIGH);
  rightRPM = -100;
  leftRPM = -100;
   delay(5000);
  Serial.println("Negative");
  Serial.println(currentLeftRPM);
  Serial.println(currentRightRPM);
  
}



void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial1.begin(115200);
  Serial2.begin(115200);

  Scheduler.startLoop(m1ctl);
  Scheduler.startLoop(m2ctl);
  Scheduler.startLoop(blink);
  delay(5000);
  setM1Speed(0);
  setM2Speed(0);
}


void loop() {
  delay(100);
}
