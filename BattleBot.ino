#define LEFT_LIMIT 0
#define RIGHT_LIMIT 180

#define GRIPPER_PIN 4
#define GRIPPER_OPEN 1500
#define GRIPPER_CLOSE 1000

#define PI 3.14159

#define DISTANCE_LIMIT_LEFT 20
#define DISTANCE_LIMIT_FORWARD 11
#define DISTANCE_LIMIT_TURN 30
#define DISTANCE_ONE_PULSE PI * 6.5 / 20  // 6.5 - wheel Diameter, 20 - pulses per wheel
#define DISTANCE_GET_DELAY 20
#define DISTANCE_WIDTH 15

#define DISTANCE_SENSOR_TRIG_PIN          8
#define DISTANCE_SENSOR_FORWARD_ECHO_PIN  7
#define DISTANCE_SENSOR_LEFT_ECHO_PIN     9

#define MOTOR_LEFT_PIN_1 11
#define MOTOR_LEFT_PIN_2 10
#define MOTOR_LEFT_SENSOR 2

#define MOTOR_RIGHT_PIN_1 5
#define MOTOR_RIGHT_PIN_2 6
#define MOTOR_RIGHT_SENSOR 3


unsigned int gripperState = GRIPPER_OPEN;

unsigned int robotState = 0;
// 0 - stopped
// 1 - turn eyes left
// 2 - going forward to obstacle

int speedLeft = 220;
int speedRight = 250;

unsigned int motorLeftSensor = 0;
unsigned int motorRightSensor = 0;


void setup() {
  Serial.begin(9600);
  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(DISTANCE_SENSOR_TRIG_PIN, OUTPUT);
  pinMode(DISTANCE_SENSOR_FORWARD_ECHO_PIN, INPUT);
  pinMode(DISTANCE_SENSOR_LEFT_ECHO_PIN, INPUT);

  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  pinMode(MOTOR_LEFT_SENSOR, INPUT);
  pinMode(MOTOR_RIGHT_SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_SENSOR), countPulseL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_SENSOR), countPulseR, CHANGE);
  gripperState = GRIPPER_OPEN;
}

void loop() {
  gripper(gripperState);
  gripper(0);
  int distanceLeft = getLeftDistance();
  delay(50);
  int distanceForward =  getForwardDistance();
 
  
  if(distanceForward > DISTANCE_LIMIT_FORWARD) {
    Serial.println("FORWARD IS FREE");
    forward();
    if (distanceLeft > DISTANCE_LIMIT_LEFT) {
      turnLeft();
      stop();
    }
  } else {
    turnRight();
    stop();
  }
  
  /*
  if (distanceLeft > DISTANCE_LIMIT_LEFT) {
      Serial.println("LEFT FREE");
      forward();
      delay(200);
      turnLeft();
    }
  if (getForwardDistance() > DISTANCE_LIMIT_FORWARD) {
    Serial.println("FORWARD");
    forward();
  } else {
    stop();
    Serial.println("STOP");
    gripperState = GRIPPER_CLOSE;
  }

  /*static unsigned int distanceToObstacle;
  if (robotState == 0) {
    getDistance();
    stop();
    delay(100);/*
    eyes(EYES_FORWARD);
    Serial.println("Turn eyes forward");
    distanceToObstacle = getDistance();
    Serial.print("Distance to Obstacle ");
    Serial.println(distanceToObstacle);
    robotState = 1;
    if (distanceToObstacle >= 1000) {
      Serial.println("Set distance to 400");
      distanceToObstacle = 400;
    } else if (distanceToObstacle < DISTANCE_LIMIT) {
      Serial.println("Distance too small, stopping...");
      stop();
      robotState = 4;
    }
  }
  if (robotState == 1) {
    Serial.println("Turn Eyes Left");
    stop();
    eyes(EYES_LEFT);
    delay(50);
    robotState = 2;
  }
  if (robotState == 2) {
    int leftDistance = getDistance();
    Serial.print("Distnace Left: ");
    Serial.print(leftDistance);
    if (leftDistance > DISTANCE_LEFT) {
      Serial.println("GOING FORWARRD FOR LEFT TURN");
      forward(ROBOT_LENGTH/2);
      if(robotState == 0)
      {
        Serial.println("Turning left");
        turnLeftSmoothly();
        stop();
      }
    } else {
        Serial.print("     Going Forward: ");
        Serial.println(distanceToObstacle);
        forward(distanceToObstacle);
    }
  }
  /*Serial.print("          State: ");
  Serial.println(robotState);


  /*Serial.print("Motor L: ");
    Serial.println(motorLeftSensor);
    
    Serial.print("Motor R: ");
    Serial.println(motorRightSensor);*/




  /*forward();
  delay(3000);
  back();
  delay(3000);
  left();
  delay(3000);
  right();
  delay(3000);*/
  /*look();
  open();
  delay(500);
  close();*/
  // lineSensorValue = ((analogRead(lsensor1)+analogRead(lsensor2))/2);
  // if (lineSensorValue >750 ) {

  //    // black
  //    // move
  //   } else if (lineSensorValue <750) {
  //     // white
  //     // move
  //   }

  //      // Добавьте другую логику, основанную на значениях датчика, здесь
  //      delay(500); // Настройте по необходимости
  //  }


  /*
    for (int pos = LEFT_LIMIT; pos <= RIGHT_LIMIT; pos += SCAN_SPEED) {
        eyes.write(pos);
        int distance = getDistance();
        if(pos <= 80 && distance >= DISTANCE_LIMIT) {
            left();
        }  else if (pos > 80 && pos < 130 && distance < DISTANCE_LIMIT) {
            back();
            delay(500);
            turn();
        }

        Serial.println(distance);
    }
    for (int pos = RIGHT_LIMIT; pos >= LEFT_LIMIT; pos -= SCAN_SPEED) {
        eyes.write(pos);
        int distance = getDistance();
        Serial.println(distance);
    }*/
}

void countPulseL() {
  motorLeftSensor++;
}

void countPulseR() {
  motorRightSensor++;
}

void stop() {
  drive(MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2, 0);
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, 0);
  motorRightSensor = 0;
  motorLeftSensor = 0;
}

void turnLeft() {
  forward();
  delay(300);
  stop();
  for (int totalPulses = 0; totalPulses < 40;) {
    left();
    totalPulses += motorRightSensor;
    motorRightSensor = 0;
    Serial.println(totalPulses);
  }
  motorRightSensor = 0;
  motorLeftSensor = 0;
  stop();
  // while(true){
  //   Serial.println("STOPPED");
  // }
}

void turnLeftSmoothly() {
  back();
  for (int totalPulses = 0; totalPulses < 55;) {
    smoothLeft();
    totalPulses += motorRightSensor;
    motorRightSensor = 0;
  }
  stop();
}

void turnRight() {
  for(int forwardDistance = getForwardDistance(); forwardDistance <= DISTANCE_LIMIT_FORWARD+5; forwardDistance = getForwardDistance()) {
      back();
  }
  stop();
  for(int forwardDistance = getForwardDistance(); forwardDistance < DISTANCE_LIMIT_TURN; forwardDistance = getForwardDistance()) {
      back();
      delay(300);
      right();
      delay(400);
  }
  stop();
}

void turn() {
  int arcLen = (110 * PI * 180 / 180) * 2;
  for (int totalPulses = 0; totalPulses < (arcLen / DISTANCE_ONE_PULSE);) {
    drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, speedLeft);
    drive(MOTOR_RIGHT_PIN_2, MOTOR_RIGHT_PIN_1, speedRight);
    totalPulses += motorRightSensor;
    motorRightSensor = 0;
    delay(10);
  }
  stop();
}

void left() {
  drive(MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2, 0);
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, 255);
}

void smoothLeft() {
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, speedLeft / 3);
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, speedRight);
}

void right_back() {
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, 0);
  drive(MOTOR_RIGHT_PIN_2, MOTOR_RIGHT_PIN_1, speedRight);
}

void right() {
  drive(MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2, 255);
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, 0);
}
void forward() {
  int speedDif = motorLeftSensor - motorRightSensor;
  speedLeft -= speedDif;
  if (speedLeft < 1) {
    speedLeft = 0;
  } else if (speedLeft > 250) {
    speedLeft = 250;
  }
  drive(MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2, speedLeft);
  drive(MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2, speedRight);
}

void back() {
  drive(MOTOR_LEFT_PIN_2, MOTOR_LEFT_PIN_1, speedLeft);
  drive(MOTOR_RIGHT_PIN_2, MOTOR_RIGHT_PIN_1, speedRight);
}

void drive(int a1, int a2, int speed) {
  analogWrite(a1, speed);
  digitalWrite(a2, LOW);
}

void sendPulse(){
    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(DISTANCE_SENSOR_TRIG_PIN, LOW);
}

int getForwardDistance() {
  sendPulse();
  long duration = pulseIn(DISTANCE_SENSOR_FORWARD_ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

int getLeftDistance() {
  sendPulse();
  long duration = pulseIn(DISTANCE_SENSOR_LEFT_ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void gripper(int pulse) {
  static unsigned long timer;
  static unsigned int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + 20;
  }
}
