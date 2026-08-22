#include <Servo.h>

Servo panServo;
Servo tiltServo;

const int LASER_PIN = 13;
const int PAN_PIN = 9;
const int TILT_PIN = 10;

const byte START_MARKER = 255;
byte packet[3];
int byteCount = 0; 
int receiving = false;

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

}

void loop() {
  while (Serial.available()> 0){
    byte inByte = Serial.read();

    if (inByte == START_MARKER){
      receiving  = true;
      byteCount = 0;
      continue;
    }

    if (receiving){
      packet[byteCount] = inByte;
      byteCount ++;

      if (byteCount == 3){
        panServo.write(packet[0]);
        tiltServo.write(packet[1]);
        digitalWrite(LASER_PIN, packet[2]> 0 ? HIGH: LOW);
        receiving = false;
      }

    }
  }

}
