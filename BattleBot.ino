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

//start
int lineCount = 0;
bool shouldPerformLineCountingAndGrabbing = true;
bool lineDetected = false;

//end
bool stopForever = false;


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
  analogWrite(motorLeftForward, 230);
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
  for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              return;
          }
        }
  leds.fill(RED, 0, 4);
  leds.show();
  analogWrite(motorLeftBackwards, 240);
  digitalWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 250);
  digitalWrite(motorRightBackwards, 0);
}
void rotateCounterAxis() {
  for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              return;
          }
        }
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
        wait(300);
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
    for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
  }
   for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
  moveForward();
  while ((countLeft < nrOfPulses && countRight < nrOfPulses) && isActive) {
    for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
    if (countLeft == lastCountLeft && countRight == lastCountRight) {  //wheel has not pulsed yet
      if (millis() > movementBuffer) {                                 //if not moved for duration
         for (int i = 0; i < 8; i++) {
            int sensorValue = analogRead(lineSensors[i]);
              if (sensorValue > calculateLineThreshold()) {
                  break;
          }
        }
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
       for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
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

  for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }

}

void getDistanceFront() {
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationFront = pulseIn(echoPinFront, HIGH);
  // Calculating the distance
  distanceFront = durationFront * 0.034 / 2;
//  Serial.println(distanceFront);
        for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
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


void performLineCountingAndGrabbing() {
  static unsigned long lastLineTime = 0;
  unsigned long currentTime = millis();

  if (lineCount < 4) {
    moveForward();
    int lineDetected = checkLines();
    if (lineDetected && currentTime - lastLineTime >= 200) {
      lineCount++;
      lastLineTime = currentTime;
    }
  }
  if (lineCount == 4 ) {
    stopRobot();
    closeGripper();

    do {
    moveForward();
    } while (!areAllSensorsBelowThreshold());
    lineCount++;
    turnLeft90Degrees();
    shouldPerformLineCountingAndGrabbing = false;
  }
}

bool areAllSensorsBelowThreshold() {
    for (int i = 0; i < 8; i++) {
        if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
            return false;
        }
    }
    return true;
}

bool areAllSensorsBiggerThreshold() {
    for (int i = 0; i < 8; i++) {
        if (analogRead(lineSensors[i]) < calculateLineThreshold()) {
            return false;
        }
    }
    return true;
}

int calculateLineThreshold() {
  static int calculatedLineThreshold = -1;
  if (calculatedLineThreshold != -1) {
    return calculatedLineThreshold;
  }

  int highestValue = 0;
  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(lineSensors[i]);
    highestValue = max(highestValue, sensorValue);
  }

  calculatedLineThreshold = highestValue + 100;
  return calculatedLineThreshold;
}

int checkLines() {
  for (int i = 0; i < 8; i++) {
    if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
      return 1;
    }
  }
  return 0;
}

void turnLeft90Degrees() {
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;
  digitalWrite(motorLeftForward, LOW);
  analogWrite(motorLeftBackwards, 255);
  analogWrite(motorRightForward, 255);
  digitalWrite(motorRightBackwards, LOW);

  do {
    sensorValue0 = analogRead(lineSensors[4]); // Middle sensor 1
    sensorValue1 = analogRead(lineSensors[2]); // Middle sensor 2 (left of middle)
    sensorValue2 = analogRead(lineSensors[0]); // Middle sensor 3 (right of middle)
    sensorValue3 = analogRead(lineSensors[1]); // Middle sensor 4
    sensorValueA7 = analogRead(lineSensors[5]); // Leftmost sensor
    sensorValueA5 = analogRead(lineSensors[6]); // Rightmost sensor

  } while (!((sensorValue0 > calculateLineThreshold() || sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() || sensorValue3 > calculateLineThreshold()) &&
             sensorValueA7 < calculateLineThreshold() && sensorValueA5 < calculateLineThreshold()));

  stopRobot();
}

void turnRight90Degrees() {
    int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValueA7, sensorValueA5;
    digitalWrite(motorRightForward, LOW);
    analogWrite(motorRightBackwards, 255);
    analogWrite(motorLeftForward, 255);
    digitalWrite(motorLeftBackwards, LOW);

    do {
        sensorValue1 = analogRead(lineSensors[2]); // Adjust if needed
        sensorValue2 = analogRead(lineSensors[0]); // Adjust if needed
        sensorValueA7 = analogRead(lineSensors[5]); // Adjust if needed
        sensorValueA5 = analogRead(lineSensors[6]); // Adjust if needed
    } while (!((sensorValue1 > calculateLineThreshold() || sensorValue2 > calculateLineThreshold() )&&
               sensorValueA7 < calculateLineThreshold() && sensorValueA5 < calculateLineThreshold()));

    stopRobot();
}


bool lineDetectedMid() {
    for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
        Serial.print(sensorValue);

        if (sensorValue > calculateLineThreshold()) {
            Serial.println(" Line Detected");
            return true;
        }
    }
    return false;
}

void followLine() {
    bool lineDetected = false;

    // Convert sensor readings to binary values based on threshold comparison
    int leftSensorBinary = analogRead(lineSensors[3]) > calculateLineThreshold() ? 1 : 0;
    int rightSensorBinary = analogRead(lineSensors[1]) > calculateLineThreshold() ? 1 : 0;

    // Check if any sensor detects the line
    for (int i = 0; i < 4; i++) {
        if (analogRead(lineSensors[i]) > calculateLineThreshold()) {
            lineDetected = true;
            break; // If any sensor detects the line, no need to check further
        }
    }

    if (lineDetected) {
        moveForward(); // If line is detected under any sensor, move forward
    } else {
        if(leftSensorBinary == rightSensorBinary){
          if(areAllSensorsBelowThreshold()){
            return;
            }else{turnRight90Degrees();}

        }
        if (leftSensorBinary == 0) {
            turnLeft();
        } else if (rightSensorBinary == 0) {
             turnRight;
        }
      }
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
  Serial.begin(9600);
}

//===[ LOOP ]============================

void loop() {
  if (stopForever) {
    return;
  }
  if (hasStarted){
    if (lineDetectedMid()) {
      Serial.print("Line Detected");
      followLine();
      if (areAllSensorsBiggerThreshold()) {
        openGripper();
        moveBackwards();
        wait(300);
        stopRobot();
        stopForever = true;
        return;
      }
    }
  else {
      getDistanceLeft();
      getDistanceFront();
      if (distanceLeft <= 20 && distanceFront >= 14) {
        for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
        moveForwardOnPulses(10);
        stopRobot();
        wait(200);
      } else if (distanceLeft > 20 && distanceFront > 14) {
        for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
        rotatePulses(50);
        stopRobot();
        wait(200);
        moveForwardOnPulses(15);
        rotatePulses(50);
      } else if (distanceLeft < 10 && distanceFront <= 15) {
        for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
        leds.fill(YELLOW, 0, 4);
        leds.show();
        getDistanceFront();
        rotateCounterAxis();
        wait(400);
        moveForwardOnPulses(10);
      } else {
        for (int i = 0; i < 8; i++) {
        int sensorValue = analogRead(lineSensors[i]);
          if (sensorValue > calculateLineThreshold()) {
              break;
          }
        }
        leds.fill(WHITE, 0, 4);
        leds.show();
        moveBackwards();
        wait(300);
        rotateCounterAxis();
        wait(300);
      }
  }
  }else {
    if (hasInitiatedStart == true) {
      if (shouldPerformLineCountingAndGrabbing) {
        performLineCountingAndGrabbing();
      }
      if (!shouldPerformLineCountingAndGrabbing) {
        hasStarted = true;
      }
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
