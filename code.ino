#include <AFMotor.h>

const int KP = 140; //245
const int KD = 50; //0
const int KI = 0;

const int MIN_SPEED = 0;
const int FORWARD_SPEED = 250;
const int MAX_SPEED = 250;

const uint8_t BASE_SPEED = 250; //125
const uint8_t TURN_SPEED = 250;

int leftSpeed, rightSpeed, lightSensor, count;
int error, P, I, D, PID, previousI, previousError, mask;

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

void setup() {
  //  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(2, OUTPUT);
}

void turnLeft() {
  motor1.run(FORWARD);
  motor1.setSpeed(TURN_SPEED);
  motor2.run(FORWARD);
  motor2.setSpeed(TURN_SPEED);
  motor3.run(BACKWARD);
  motor3.setSpeed(TURN_SPEED);
  motor4.run(BACKWARD);
  motor4.setSpeed(TURN_SPEED);
}

void turnRight() {
  motor3.run(FORWARD);
  motor3.setSpeed(TURN_SPEED);
  motor4.run(FORWARD);
  motor4.setSpeed(TURN_SPEED);
  motor1.run(BACKWARD);
  motor1.setSpeed(TURN_SPEED);
  motor2.run(BACKWARD);
  motor2.setSpeed(TURN_SPEED);
}

void moveForward() {
  motor1.run(FORWARD);
  motor1.setSpeed(FORWARD_SPEED);
  motor2.run(FORWARD);
  motor2.setSpeed(FORWARD_SPEED);
  motor3.run(FORWARD);
  motor3.setSpeed(FORWARD_SPEED);
  motor4.run(FORWARD);
  motor4.setSpeed(FORWARD_SPEED);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor1.setSpeed(FORWARD_SPEED);
  motor2.run(BACKWARD);
  motor2.setSpeed(FORWARD_SPEED);
  motor3.run(BACKWARD);
  motor3.setSpeed(FORWARD_SPEED);
  motor4.run(BACKWARD);
  motor4.setSpeed(FORWARD_SPEED);
}

void stop() {
  motor1.run(RELEASE);
  motor1.setSpeed(0);
  motor2.run(RELEASE);
  motor2.setSpeed(0);
  motor3.run(RELEASE);
  motor3.setSpeed(0);
  motor4.run(RELEASE);
  motor4.setSpeed(0);
}


void readSensor() {
  mask ^= mask;
  mask |= digitalRead(A0) << 0;
  mask |= digitalRead(A1) << 1;
  mask |= digitalRead(A2) << 2;
  mask |= digitalRead(A3) << 3;
  mask |= digitalRead(A4) << 4;
  lightSensor = digitalRead(A5);
  calcLight();
}

void calcLight() {
  if (lightSensor) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }
}

void calcError() {
    switch (mask) {
    case 0b01000 : error = 3; break;
    case 0b01100 : error = 2; break;

    case 0b01110 : error = 0; break;
    case 0b00100 : error = 0; break;

    case 0b00110 : error = -2; break;
    case 0b00010 : error = -3; break;

    case 0b00101 :
    case 0b01101 :
    case 0b01111 :
    case 0b00111 :
    case 0b00011 :
    case 0b00001 : error = 102; break; //90 degree right

    case 0b10100 :
    case 0b10110 :
    case 0b10000 :
    case 0b11000 :
    case 0b11100 :
    case 0b11110 : error = 103; break; //90 degree left

    case 0b11111 : error = 100; break; // Stop the car, all black line or nothing
    case 0b00000 : error = 101; break; // Move forward, no line at all
    default : error = 404; break;
  }
  
}

void calcPid() {
  P = error;
  I = I + previousI;
  I = max(I, 0);
  D = error - previousError;

  PID = (KP * P) + (KI * I) + (KD * D);

  previousI = I;
  previousError = error;
}

int castSpeed(int speed) {
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  return speed;
}

void calcMotor() {
  switch (error) {
    case 404: stop(); return;
    case 100: stop(); return;
    case 101: moveForward(); return;
    case 102: {
        do {
          readSensor();
          calcError();          
          turnRight();
          //          delay(1);
        } while (error != 0 && error != 100);
        return;
      }
    case 103: {
        do {
          readSensor();
          calcError();
          turnLeft();
          //          delay(1);
        } while (error != 0 && error != 100);
        return;
      }
    default:
      rightSpeed = BASE_SPEED + PID;
      leftSpeed = BASE_SPEED - PID;

      if (leftSpeed < 0) {
        leftSpeed = castSpeed(abs(leftSpeed));
        motor3.run(BACKWARD);
        motor4.run(BACKWARD);
        motor3.setSpeed(leftSpeed);
        motor4.setSpeed(leftSpeed);
      } else {
        leftSpeed = castSpeed(leftSpeed);
        motor3.run(FORWARD);
        motor4.run(FORWARD);
        motor3.setSpeed(leftSpeed);
        motor4.setSpeed(leftSpeed);
      }

      if (rightSpeed < 0) {
        rightSpeed = castSpeed(abs(rightSpeed));
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
        motor1.setSpeed(rightSpeed);
        motor2.setSpeed(rightSpeed);
      } else {
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        rightSpeed = castSpeed(rightSpeed);
        motor1.setSpeed(rightSpeed);
        motor2.setSpeed(rightSpeed);
      }
  }
}

void loop() {
  readSensor();
  calcError();
  calcPid();
  calcMotor();
  //  delay(1);
}
