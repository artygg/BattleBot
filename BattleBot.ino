// Define pins for motor 1
int pwmPinMotor1 = 3; // PWM pin for motor 1
int dirPinMotor1 = 4; // Direction pin for motor 1

// Define pins for motor 2
int pwmPinMotor2 = 6; // PWM pin for motor 2
int dirPinMotor2 = 5; // Direction pin for motor 2

void setup() {
    // Initialize PWM and direction pins as outputs
    pinMode(pwmPinMotor1, OUTPUT);
    pinMode(dirPinMotor1, OUTPUT);
    pinMode(pwmPinMotor2, OUTPUT);
    pinMode(dirPinMotor2, OUTPUT);
}

void loop() {
    // Set direction and PWM duty cycle for motor 1 (e.g., half-speed forward)
    digitalWrite(dirPinMotor1, HIGH); // Set direction forward
    analogWrite(pwmPinMotor1, 250); // Example: 50% duty cycle for motor 1

    // Set direction and PWM duty cycle for motor 2 (e.g., quarter-speed backward)
    digitalWrite(dirPinMotor2, HIGH); // Set direction backward
    analogWrite(pwmPinMotor2, 250); // Example: 25% duty cycle for motor 2

    delay(1000); // Delay for 1 second
}
