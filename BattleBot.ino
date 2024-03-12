#define LEFT_LIMIT   0
#define RIGHT_LIMIT  180
#define SCAN_SPEED   60
#define GRIPPER_OPEN 1500
#define GRIPPER_CLOSE 1000
#define EYES_FORWARD 1400
#define EYES_LEFT 2300
#define EYES_RIGHT 500

const int distanceLimit = 10;
const float pi = 3.141592653589793238462643383279;

//Distance from eyes to the back wheel - 180mm
//Distance between wheels - 110mm

// Define pins for motor 1
const int a1Motor1 = 11; // PWM pin for motor 1
const int a2Motor1 = 10; // Direction pin for motor 1
const int sensorPinMotor1 = 2; //Sensor pin for motor 1

// Define pins for motor 2
const int a1Motor2 = 5; // PWM pin for motor 2
const int a2Motor2 = 6; // Direction pin for motor 2
const int sensorPinMotor2 = 3; //Sensor pin for motor 2

const float pulseDistance = pi*65/20; //Wheel diameter in mm * PI constant and divided by pulses per 1 spin

const int gripperPin = 4;
// <<<<<<< HEAD
const int distanceRotatorPin = 12;
// =======
const int eyesPin = 12;
// >>>>>>> 1acf7c1be2bb43fe18226e1638d8e9593366a886

const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int blackLineCount = 0;

bool lineFollowingMode = false; // Flag for line following mode

int lineSensorValue;
bool isstopped = true;

const int trigPin = 8;  
const int echoPin = 7; 

int speed1 = 255;
int speed2 = 255;


int sensorValueMotorL = 0;
int sensorValueMotorR = 0;

int totalPulses = 0;

volatile unsigned long pulseCountL = 0;
unsigned long lastTimeL = 0; 

bool shouldPerformStart = true;


void setup() {

    // Initialize PWM and direction pins as outputs
    pinMode(gripperPin, OUTPUT);
    // pinMode(eyesPin, OUTPUT);
    pinMode(trigPin, OUTPUT);  
  	pinMode(echoPin, INPUT);  
	  Serial.begin(9600);  



    pinMode(a1Motor1, OUTPUT);
    pinMode(a2Motor1, OUTPUT);
    pinMode(a1Motor2, OUTPUT);
    pinMode(a2Motor2, OUTPUT);
    pinMode(sensorPinMotor1, INPUT);
    pinMode(sensorPinMotor2, INPUT);
    attachInterrupt(digitalPinToInterrupt(sensorPinMotor2), countPulseL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensorPinMotor1), countPulseR, CHANGE);
// <<<<<<< HEAD

    // gripper.attach(gripperPin);
//     eyes.attach(distanceRotatorPin);
// =======
// >>>>>>> 1acf7c1be2bb43fe18226e1638d8e9593366a886
    //pinMode(sensorPinMotor1, INPUT_PULLUP);
    //pinMode(sensorPinMotor2, INPUT_PULLUP);
    Serial.println(pulseDistance);

    for (int i = 0; i < 8; i++) {
    pinMode(lineSensorPins[i], INPUT);

    gripper(GRIPPER_OPEN);

  }

}

void loop() {  
  ////////////////////////////////////
  if (shouldPerformStart){
    int sensorValues[8];
    for (int i = 0; i < 8; i++) {
      sensorValues[i] = digitalRead(lineSensorPins[i]);
    }

  // Check if all sensors detect black
    bool allSensorsBlack = true;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] <700) { // Assuming HIGH means no black line detected
        allSensorsBlack = false;
        break;
      }
    }

    if (allSensorsBlack) {
      blackLineCount++;
      stop(); // Stop to count this as a distinct black line detection
      delay(500); // Adjust timing as necessary

      if (blackLineCount == 4) {

        gripper(GRIPPER_CLOSE);
        blackLineCount = 0; // Reset count or modify logic as needed
        // Exit line following mode
      }
      else if (blackLineCount < 4) {
        forward(); // Enter line following mode
      }
    }else {
      // Default behavior: move forward
      forward();
    }
    // else if (lineFollowingMode && blackLineCount == 3) {
    //   // Follow the line using specific sensors, e.g., 4th and 5th
    //   followLine(sensorValues[3], sensorValues[4]);
    // }
    
    shouldPerformStart = false;

  }
  
    // gripper(GRIPPER_CLOSE);
    // stop();
    // eyes(EYES_LEFT);
    // delay(2000);
    // eyes(EYES_RIGHT);
    // delay(2000);
    // eyes(EYES_FORWARD);
    // delay(300);
    // int distanceToObstacle = getDistance();
    // Serial.println(distanceToObstacle);
    // delay(50);
    // //eyes.write(180);
    // delay(300);
    // if(distanceToObstacle >= 1000) {
    //   distanceToObstacle = 30;
    // }
    // for(int totalPulses = 0; totalPulses<(distanceToObstacle - distanceLimit)/pulseDistance;) {
    //   Serial.println("ok");
    //   forward();
    //   int leftDistance = getDistance();
    //   delay(100);
    //   if(leftDistance > 200) {
    //     stop();
    //     turnLeft(90);
    //   }
    //   int speedDif = sensorValueMotorL - sensorValueMotorR;
    //   speed1 -= speedDif*5;
    //   if (speed1<1) {
    //     speed1 = 0;
    //   } else if(speed1>250) {
    //     speed1 = 250;
    //   }
    //   totalPulses += sensorValueMotorR;
    //   sensorValueMotorL = 0;
    //   sensorValueMotorR = 0;
    //   delay(50); 
    // }
    // totalPulses = 0;
    ////////////////////////////////////////
 
    /*Serial.print("Motor L: ");
    Serial.println(sensorValueMotorL);
    
    Serial.print("Motor R: ");
    Serial.println(sensorValueMotorR);*/
  
  

  
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
    sensorValueMotorL++;
}

void countPulseR() {
    sensorValueMotorR++;
}

void stop(){
  isstopped = true;
  drive(a1Motor1, a2Motor1, 0);
  drive(a1Motor2, a2Motor2, 0);
}

void turnLeft(int deg) {
  int arcLen = (110*pi*deg/180)*2;
  for(int totalPulses = 0;totalPulses<(arcLen/pulseDistance);) {
      left();
      totalPulses += sensorValueMotorR;
      sensorValueMotorR = 0; 
      delay(10);
  }
  stop();
  delay(1000);
}

void turnRight(int deg) {
  int arcLen = (110*pi*deg/180)*2;
  for(int totalPulses = 0;totalPulses<(arcLen/pulseDistance);) {
      right_back();
      totalPulses += sensorValueMotorR;
      sensorValueMotorR = 0;
      delay(10);
  }
  stop();
}

void turn() {
  int arcLen = (110*pi*180/180)*2;
  for(int totalPulses = 0;totalPulses<(arcLen/pulseDistance);) {
      drive(a1Motor1, a2Motor1, speed1);
      drive(a2Motor2, a1Motor2, speed2);
      totalPulses += sensorValueMotorR;
      sensorValueMotorR = 0;
      delay(10);
  }
  stop();
}

void left() {
  drive(a1Motor1, a2Motor1, 0);
  drive(a1Motor2, a2Motor2, speed2);
}
void right_back() {
  drive(a1Motor1, a2Motor1, 0);
  drive(a2Motor2, a1Motor2, speed2);
}



void right() {
  drive(a1Motor1, a2Motor1, speed1);
  drive(a1Motor2, a2Motor2, 0);
}

void forward() {
  if(isstopped) {
    drive(a1Motor1, a2Motor1, 200);
    drive(a1Motor2, a2Motor2, 200);
    delay(100);
    isstopped = false;
  }
  
  drive(a1Motor1, a2Motor1, speed1);
  drive(a1Motor2, a2Motor2, speed2);
}
void back() {
  drive(a2Motor1, a1Motor1, speed1);
  drive(a2Motor2, a1Motor2, speed2);
}

void drive(int a1, int a2, int speed) {
  analogWrite(a1, speed);
  
}


int getDistance()  {
  int total = 0;
  for(int i = 0; i<10;i++) {
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    int distance = (duration*0.0343)/2;
    total += distance;
  }
  return total;
}


/*void gripper(int pulse) {
  static unsigned long timer;
  static unsigned int pulse1;
  if(pulse > 0) {
    pulse1 = pulse;
  }
  if(millis() > timer) {
      digitalWrite(gripperPin, HIGH);
      delayMicroseconds(pulse1);
      digitalWrite(gripperPin, LOW);
      timer = millis() + 20;
  }
}*/



  void followLine(int sensorLeft, int sensorRight) {
  // Basic line following logic, adjust as needed for your specific sensor setup and robot dynamics
  if (sensorLeft == LOW && sensorRight == HIGH) {
    // Adjust to turn towards the line
    digitalWrite(a1Motor1, LOW);
    digitalWrite(a1Motor2, HIGH);
  } else if (sensorLeft == HIGH && sensorRight == LOW) {
    // Adjust to turn towards the line
    digitalWrite(a1Motor1, HIGH);
    digitalWrite(a1Motor2, LOW);

  }
  }

  void gripper(int pulse) {
  static unsigned long timer;
  static int pulsel;
  if (pulse > 0){
    pulsel = pulse;
  }
  if (millis() > timer){
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulsel);
    digitalWrite(gripperPin, LOW);
    timer = millis() + 20;
    
  }

  // for(int i = 0; i < 10; i++) {
  //   digitalWrite(gripperPin, HIGH);
  //   delayMicroseconds(pulse);
  //   digitalWrite(gripperPin, LOW);
  //   delay(20);
  // }
}

void eyes(int pulse) {
  for(int i = 0; i < 10; i++) {
    // digitalWrite(eyesPin, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(eyesPin, LOW);
    delay(20);
  }
}