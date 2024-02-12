#include <Servo.h>

#define LEFT_LIMIT   0
#define RIGHT_LIMIT  180
#define SCAN_SPEED   60
#define DISTANCE_LIMIT 30

// Define pins for motor 1
const int a1Motor1 = 11; // PWM pin for motor 1
const int a2Motor1 = 10; // Direction pin for motor 1
const int sensorPinMotor1 = 8; //Sensor pin for motor 1

// Define pins for motor 2
const int a1Motor2 = 5; // PWM pin for motor 2
const int a2Motor2 = 6; // Direction pin for motor 2
const int sensorPinMotor2 = 9; //Sensor pin for motor 2

const int lsensor1 = A0;
const int lsensor2=A1;

int sensorValue;

bool inverted = false;

const int trigPin = 2;  
const int echoPin = 7; 


Servo gripper;
Servo eyes;

const int speed1 = 235;
const int speed2 = 250;


float duration, distance;  
int sensorValueMotor1 = 0;
int sensorValueMotor2 = 0;



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
    gripper.attach(3);
    eyes.attach(12);
    //pinMode(sensorPinMotor1, INPUT_PULLUP);
    //pinMode(sensorPinMotor2, INPUT_PULLUP);
    open();
    delay(4000);
    close();
    

}

void loop() {
  forward();
  /*
    sensorValueMotor1 = digitalRead(sensorPinMotor1);
    sensorValueMotor2 = digitalRead(sensorPinMotor2);
    int speedAdjustment = 0;
    if (sensorValueMotor1 == HIGH && sensorValueMotor2 == LOW) {
        speedAdjustment = 5; // Increase speed of motor 2
    } else if (sensorValueMotor1 == LOW && sensorValueMotor2 == HIGH) {
        speedAdjustment = -5; // Decrease speed of motor 2
    }*/
    //forward(a1Motor1, a2Motor1, 235);
    //forward(a1Motor2, a2Motor2, 250);
    //Serial.println(speedAdjustment);

    /*digitalWrite(trigPin, LOW);  
	  delayMicroseconds(2);  
	  digitalWrite(trigPin, HIGH);  
	  delayMicroseconds(10);  
	  digitalWrite(trigPin, LOW);  


    Serial.print("Distance: ");  
    Serial.println(distance);  
    delay(100); 
    if(distance < 30) {
      back();
      delay(3000);
      right();
      delay(500);
    }*/

  
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
  sensorValue = ((analogRead(lsensor1)+analogRead(lsensor2))/2);
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
    }
}



void stop(){
  drive(a1Motor1, a2Motor1, 0);
  drive(a1Motor2, a2Motor2, 0);
}

void left() {
  drive(a1Motor1, a2Motor1, 0);
  drive(a1Motor2, a2Motor2, 250);
}

void right() {
  drive(a1Motor1, a2Motor1, 235);
  drive(a1Motor2, a2Motor2, 0);
}

void forward() {
  drive(a1Motor1, a2Motor1, 235);
  drive(a1Motor2, a2Motor2, 250);
}
void back() {
  drive(a2Motor1, a1Motor1, 235);
  drive(a2Motor2, a1Motor2, 250);
}

void drive(int a1, int a2, int speed) {
  digitalWrite(a1, speed);
  digitalWrite(a2, LOW);
}

void open(){
  gripper.write(120);
}

void close(){
  gripper.write(0);
}

int getDistance()  {
    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    return distance;
}

void turn(){
    eyes.write(80);
    for(;distance < DISTANCE_LIMIT;) {
        distance = getDistance();
        if(distance < 10) {
            right();
        } else {
            back();
        }
    }
}
