#include <Servo.h>

#define LEFT_LIMIT   0
#define RIGHT_LIMIT  180
#define SCAN_SPEED   60
const int distanceLimit 5;
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

const int lsensor1 = A0;
const int lsensor2 = A1;

int lineSensorValue;
bool isstopped = true;

const int trigPin = 8;  
const int echoPin = 7; 


Servo gripper;
Servo eyes;

int speed1 = 230;
int speed2 = 250;


int sensorValueMotorL = 0;
int sensorValueMotorR = 0;

int totalPulses = 0;

volatile unsigned long pulseCountL = 0;
unsigned long lastTimeL = 0; 


void setup() {
    // Initialize PWM and direction pins as outputs
    pinMode(trigPin, OUTPUT);  
  	pinMode(echoPin, INPUT);  
	  Serial.begin(9600);  

    pinMode(lsensor1, INPUT);
    pinMode(lsensor2, INPUT);

    pinMode(a1Motor1, OUTPUT);
    pinMode(a2Motor1, OUTPUT);
    pinMode(a1Motor2, OUTPUT);
    pinMode(a2Motor2, OUTPUT);
    pinMode(sensorPinMotor1, INPUT);
    pinMode(sensorPinMotor2, INPUT);
    attachInterrupt(digitalPinToInterrupt(sensorPinMotor2), countPulseL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensorPinMotor1), countPulseR, CHANGE);

    gripper.attach(gripperPin);
    eyes.attach(12);
    //pinMode(sensorPinMotor1, INPUT_PULLUP);
    //pinMode(sensorPinMotor2, INPUT_PULLUP);
    open();
    delay(1000);
    close();
    Serial.println(pulseDistance);

}

void loop() {
    eyes.write(85);
    delay(300);
    int distanceToObstacle = getDistance();
    delay(200);
    eyes.write(180);
    /*Serial.println(totalPulses);
    Serial.println(distanceToObstacle);
    Serial.println((distanceToObstacle - DISTANCE_LIMIT)/pulseDistance);*/
    for(int totalPulses = 0; totalPulses<(distanceToObstacle - DISTANCE_LIMIT)/pulseDistance;) {
      forward();
      int speedDif = sensorValueMotorL - sensorValueMotorR;
      speed1 -= speedDif*5;
      if (speed1<1) {
        speed1 = 0;
      } else if(speed1>250) {
        speed1 = 250;
      }
      totalPulses += sensorValueMotorR;
      sensorValueMotorL = 0;
      sensorValueMotorR = 0;
      delay(50); 
    }
    totalPulses = 0;
    turnRight(180);
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
  delay(1000);
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
    delay(50);
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
  digitalWrite(a2, LOW);
}

void open(){
  gripper.write(120);
}

void close(){
  gripper.write(50);
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

void turn(){
    drive(a1Motor1, a2Motor1, speed1);
    drive(a2Motor2, a1Motor2, speed2);
    
}
