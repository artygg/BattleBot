#include <Adafruit_NeoPixel.h>

boolean hasInitiatedStart = false;
boolean hasStarted = false;

const int lineSensors[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int lineSensorSensitivity = 700;

const int GRIPPER_PIN = 4;
const int GRIPPER_OPEN_PULSE = 1500;
const int GRIPPER_CLOSE_PULSE = 1000;
const int GRIPPER_PULSE_REPEAT = 10;

unsigned long time;

long durationFront;
int distanceFront;
const int trigPinFront = 8;
const int echoPinFront = 7;
int backCounter = 0;

long durationLeft;
int distanceLeft;
const int trigPinLeft = 12;
const int echoPinLeft = 9;
int distanceOverride = 0;

//===[ Motor pins ]======================

const int motorRightBackwards = 6;
const int motorRightForward = 5;
const int motorLeftBackwards = 10;
const int motorLeftForward = 11;
const int movementStuckBufferDelay = 1000;

//===[ Motor tests for pulses ]====================

const int motor_R1 = 3;
const int motor_R2 = 2;
int countLeft = 0;
int countRight = 0;
int countsLeft = 0, previousCountLeft;
int countsRight = 0, previousCountRight;


//===[ Led Pixels ]================================

const int PIXEL_PIN = 13;
const int PIXEL_NUMBER = 4;
Adafruit_NeoPixel leds(PIXEL_NUMBER, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
const uint32_t RED = leds.Color(255, 0, 0);
const uint32_t YELLOW = leds.Color(255, 150, 0);
const uint32_t BLUE = leds.Color(0, 0, 255);
const uint32_t WHITE = leds.Color(255, 255, 255);
const uint32_t START = leds.Color(0, 0, 0);
//===[ Functions ]=================================

boolean isBlackZone() {
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (analogRead(lineSensors[i]) > lineSensorSensitivity) {
      count++;
    }
  }
  if (count < 4) {
    return false;
  }
  return true;
}

void adjustRight() {
  turnRight();
  wait(200);
  moveForward();
  wait(200);
  turnLeft();
  wait(200);
  moveForward();
  wait(200);
  turnRight();
  wait(100);
}

void adjustLeft() {
  turnLeft();
  wait(200);
  moveForward();
  wait(200);
  turnRight();
  wait(200);
  moveForward();
  wait(200);
  turnLeft();
  wait(100);
}

void setup_motor_pins() {
  pinMode(motorRightBackwards, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorLeftBackwards, OUTPUT);
  pinMode(motorLeftForward, OUTPUT);
}
void moveForward() {
  leds.fill(BLUE, 0, 4);
  leds.show();
  analogWrite(motorLeftForward, 240);
  analogWrite(motorRightForward, 250);
  digitalWrite(motorRightBackwards, 0);
  digitalWrite(motorLeftBackwards, 0);
}

void moveBackwardsRotate() {
  analogWrite(motorRightBackwards, 255);
  analogWrite(motorLeftBackwards, 120);
  digitalWrite(motorRightForward, 0);
  digitalWrite(motorLeftForward, 0);
}

void moveBackwards() {
  analogWrite(motorRightBackwards, 255);
  analogWrite(motorLeftBackwards, 200);
  digitalWrite(motorRightForward, 0);
  digitalWrite(motorLeftForward, 0);
}

void stopRobot() {
  digitalWrite(motorRightBackwards, 0);
  digitalWrite(motorRightForward, 0);
  digitalWrite(motorLeftBackwards, 0);
  digitalWrite(motorLeftForward, 0);
}
void turnLeft() {
  leds.fill(RED, 0, 4);
  leds.show();
  digitalWrite(motorLeftBackwards, 0);
  digitalWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 250);
  digitalWrite(motorRightBackwards, 0);
}

void turnRight() {
  leds.fill(RED, 0, 4);
  leds.show();
  digitalWrite(motorLeftBackwards, 0);
  digitalWrite(motorLeftForward, 250);
  analogWrite(motorRightForward, 0);
  digitalWrite(motorRightBackwards, 0);
}
void rotateOnAxis() {
  leds.fill(RED, 0, 4);
  leds.show();
  analogWrite(motorLeftBackwards, 240);
  digitalWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 250);
  digitalWrite(motorRightBackwards, 0);
}
void rotateCounterAxis() {
  digitalWrite(motorLeftBackwards, 0);
  analogWrite(motorLeftForward, 240);
  digitalWrite(motorRightForward, 0);
  analogWrite(motorRightBackwards, 250);
}
void rotatePulses(int nrOfPulses) {
  stopRobot();
  countLeft = 0;
  countRight = 0;
  int lastCountRight = -1;
  int lastCountLeft = -1;
  long movementBuffer = millis() + movementStuckBufferDelay;
  boolean isActive = true;
  getDistanceLeft();
  turnLeft();
  wait(100);
  while ((countLeft < nrOfPulses && countRight < nrOfPulses) && distanceLeft >= 15 && isActive) {
    if (countLeft == lastCountLeft && countRight == lastCountRight) {  //wheel has not pulsed yet
      if (millis() > movementBuffer) {                                 //if not moved for duration
        movementBuffer = millis() + movementStuckBufferDelay;
        leds.fill(WHITE, 0, 4);
        leds.show();
        moveBackwardsRotate();
        wait(450);        
      }
    } else {
      movementBuffer = millis() + movementStuckBufferDelay;
      lastCountLeft = countLeft;
      lastCountRight = countRight;
    }
    getDistanceLeft();
    getDistanceFront();
  }
  stopRobot();
}

void moveForwardOnPulses(int nrOfPulses) {
  getDistanceLeft();
  stopRobot();
  countLeft = 0;
  countRight = 0;
  int lastCountRight = -1;
  int lastCountLeft = -1;
  long movementBuffer = millis() + movementStuckBufferDelay;
  boolean isActive = true;
  
  if(distanceLeft < 5) {
    adjustRight();
  } 
  moveForward();
  while ((countLeft < nrOfPulses && countRight < nrOfPulses) && isActive) {
    if (countLeft == lastCountLeft && countRight == lastCountRight) {  //wheel has not pulsed yet
      if (millis() > movementBuffer) {                                 //if not moved for duration
        movementBuffer = millis() + movementStuckBufferDelay;
        leds.fill(WHITE, 0, 4);
        leds.show();
        moveBackwards();
        wait(300);
        if(countLeft - lastCountLeft < 3 || countRight - lastCountRight < 3 ) {
          turnLeft();
          wait(100);
        }
      }
    } else {
      movementBuffer = millis() + movementStuckBufferDelay;
      lastCountLeft = countLeft;
      lastCountRight = countRight;
    }
    getDistanceFront();
  }
  stopRobot();
}
void showNrOfPulse() {
  Serial.print(countLeft);
  Serial.print(" ");
  Serial.print(countRight);
  Serial.println();
}
void CountA() {
  noInterrupts();
  countLeft++;
  interrupts();
}

void CountB() {
  noInterrupts();
  countRight++;
  interrupts();
}

void getDistanceLeft() {
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationLeft = pulseIn(echoPinLeft, HIGH);
  // Calculating the distance
  distanceLeft = durationLeft * 0.034 / 2;
}

void getDistanceFront() {
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationFront = pulseIn(echoPinFront, HIGH);
  // Calculating the distance
  distanceFront = durationFront * 0.034 / 2;
  Serial.println(distanceFront);
}

void wait(int waitingTime) {
  time = millis();
  while (millis() < time + waitingTime) {
  }
}

void gripperServo(int pulse) {
  for (int i = 0; i < GRIPPER_PULSE_REPEAT; i++) {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(20);
  }
}

void openGripper() {
  gripperServo(GRIPPER_OPEN_PULSE);
}
void closeGripper() {
  gripperServo(GRIPPER_CLOSE_PULSE);
}
//===[SETUP ]============================

void setup() {
  pinMode(motor_R1, INPUT_PULLUP);
  pinMode(motor_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_R1), CountA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_R2), CountB, CHANGE);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT);   // Sets the echoPin as an Input
  Serial.begin(9600);
  leds.begin();
  leds.fill(BLUE, 0, 4);
  leds.show();
  countLeft = 0;
  countRight = 0;
}

//===[ LOOP ]============================

void loop() {
  if (hasStarted) {
    getDistanceLeft();
    getDistanceFront();
    if (distanceLeft <= 20 && distanceFront >= 14) {
      moveForwardOnPulses(10);
      stopRobot();
      wait(200);
    } else if (distanceLeft > 20 && distanceFront > 14) {
      rotatePulses(50);
      stopRobot();
      wait(200);
      moveForwardOnPulses(15);
      rotatePulses(50);
    } else if (distanceLeft < 10 && distanceFront <= 15) {
      leds.fill(YELLOW, 0, 4);
      leds.show();
      getDistanceFront();
      rotateCounterAxis();
      wait(400);
      moveForwardOnPulses(10);
    } else {
      leds.fill(WHITE, 0, 4);
      leds.show();
      moveBackwards();
      wait(300);
      rotateCounterAxis();
      wait(300);
    }
  } else {
    if (hasInitiatedStart == true) {
      moveForward();
      wait(800);
      closeGripper();
      countLeft = 0;
      countRight = 0;
      rotatePulses(120);
      delay(100);
      moveForwardOnPulses(70);
      hasStarted = true;
    } else {
      wait(1000);
      getDistanceFront();
      if (distanceFront < 30) {
        openGripper();
        wait(2000);
        hasInitiatedStart = true;
      }
    }
  }
}
