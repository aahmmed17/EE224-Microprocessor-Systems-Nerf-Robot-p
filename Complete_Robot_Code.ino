#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>

// FXOS8700 I2C Address
#define FXOS8700_ADDRESS 0x1E  
#define CTRL_REG1 0x2A
#define OUT_X_MSB 0x01

// Motor Control Pins
#define ENA 7 // PWM Speed Control
#define IN1 6  // Direction Control
#define IN2 5  // Direction Control

#define LED_LEFT 9 // Left LED for backward
#define LED_RIGHT 8  // Right LED for forward

#define JOYSTICK_PIN A6  // AIM
#define SERVO_PIN 12

#define BUTTON_PIN_FIRE 4  //FIRE
#define MICRO_SERVO_PIN 10

int16_t accelX, accelZ;
float tiltAngle;
int motorSpeed;  // Speed (0-255)
float tiltThreshold = 15.0;  // Minimum angle to move
float speedThresholds[] = {15, 20, 30, 40, 50};  // Tilt angle thresholds for speed levels
int speedLevels[] = {10, 20, 40, 50, 90};  // Corresponding motor speed values
int lightLevels[] = {50, 100, 150, 200, 255};  // Corresponding LED brightness levels

Servo turretServo;
int turretServoPosition = 95;

Servo pusherServo;
int pusherServoPosition = 0;
boolean readyToFire = true;

void setup() {

    Wire.begin();  // Uses A4 (SDA) & A5 (SCL) 

    // Motor pin setup
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // LED pin setup for brightness control
    pinMode(LED_LEFT, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);

    // Check if FXOS8700 is connected
    Wire.beginTransmission(FXOS8700_ADDRESS);
    if (Wire.endTransmission() != 0) {
        Serial.println("ERROR: FXOS8700 not detected! Check wiring.");
        while (1);  // Stop execution
    } else {
        Serial.println("FXOS8700 detected.");
    }

    // Activate FXOS8700
    Wire.beginTransmission(FXOS8700_ADDRESS);
    Wire.write(CTRL_REG1);
    Wire.write(0x0D);  
    Wire.endTransmission();

    pinMode(JOYSTICK_PIN, INPUT);
    turretServo.attach(SERVO_PIN);

    Serial.begin(115200);
    pinMode(BUTTON_PIN_FIRE, INPUT);
    pusherServo.attach(MICRO_SERVO_PIN);
}

void loop() {
    readAccelData();
    tiltAngle = atan2((float)accelX, (float)accelZ) * 180.0 / M_PI;

    //Serial.print("Tilt Angle: ");
    //Serial.print(tiltAngle, 2);
    //Serial.print("°  ");

    // Determine motor speed based on tilt angle
    motorSpeed = getMotorSpeed(tiltAngle);

    // Map motor speed to LED brightness
    int leftLedBrightness = map(motorSpeed, 0, 255, 0, lightLevels[getSpeedLevel(motorSpeed)]);
    int rightLedBrightness = map(motorSpeed, 0, 255, 0, lightLevels[getSpeedLevel(motorSpeed)]);

    // Move motor based on tilt direction
    if (tiltAngle > tiltThreshold) {
        moveForward();
        analogWrite(LED_RIGHT, rightLedBrightness); // Adjust brightness of right LED
        analogWrite(LED_LEFT, 0);  // Turn off left LED
    } else if (tiltAngle < -tiltThreshold) {
        moveBackward();
        analogWrite(LED_LEFT, leftLedBrightness); // Adjust brightness of left LED
        analogWrite(LED_RIGHT, 0);  // Turn off right LED
    } else {
        stopMotor();
        analogWrite(LED_LEFT, 0);  // Turn off left LED
        analogWrite(LED_RIGHT, 0); // Turn off right LED
    }

    int input = analogRead(JOYSTICK_PIN);  // Read button state
    Serial.println(input);
    if (input < 3900) {  // Button is pressed (pulled to HIGH)
        Serial.println("Button Pressed! Right!");
        if(turretServoPosition > 0){
          turretServoPosition = turretServoPosition - 5;
          turretServo.write(turretServoPosition);
          delay(5);
          Serial.println(turretServoPosition);
        }
    }
    else if (input > 4094) {  // Button is pressed (pulled to HIGH)
        Serial.println("Button Pressed! Left!");
        if(turretServoPosition < 180){
          turretServoPosition = turretServoPosition + 5;
          turretServo.write(turretServoPosition);
          delay(5);
          Serial.println(turretServoPosition);
        }
    }
    delay(10);

    int fireButtonState = digitalRead(BUTTON_PIN_FIRE);  // Read button state 
    if (fireButtonState == HIGH && readyToFire) {  // Button is pressed (pulled to HIGH)
        Serial.println("Button Pressed! Fire!");
        readyToFire = false;
        while(pusherServoPosition < 180){
          pusherServoPosition = pusherServoPosition + 10;
          pusherServo.write(pusherServoPosition);
          delay(10);
          Serial.println(pusherServoPosition);

        } 
        // pusherServoPosition = 180;
        // pusherServo.write(pusherServoPosition);
        // delay(10);
        // Serial.println(pusherServoPosition);


          delay(200);                                                                                                                                                                                                                                                                      
          while(pusherServoPosition > 0){
          pusherServoPosition = pusherServoPosition - 15;
          pusherServo.write(pusherServoPosition);
          delay(10);
          Serial.println(pusherServoPosition);
          
        }
        readyToFire = true;
    }
    delay(200);

}

// Read Accelerometer Data
void readAccelData() {
    Wire.beginTransmission(FXOS8700_ADDRESS);
    Wire.write(OUT_X_MSB);
    Wire.endTransmission(false);
    Wire.requestFrom(FXOS8700_ADDRESS, 6);

    if (Wire.available() == 6) {
        accelX = (Wire.read() << 8 | Wire.read()) >> 2;  
        if (accelX > 8191) accelX -= 16384;

        Wire.read(); Wire.read();  // Skip Y

        accelZ = (Wire.read() << 8 | Wire.read()) >> 2;
        if (accelZ > 8191) accelZ -= 16384;
    } else {
        Serial.println("Error: No I2C data received!");
    }
}

// Motor Control Functions
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeed);
//    Serial.print("Moving Forward at speed ");
//    Serial.println(motorSpeed);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motorSpeed);
//    Serial.print("Moving Backward at speed ");
//    Serial.println(motorSpeed);
}

void stopMotor() {
    analogWrite(ENA, 0);
    //Serial.println("Motor Stopped");
}

// Function to get motor speed based on tilt angle
int getMotorSpeed(float angle) {
    int speed = 0;

    // Adjust speed based on the tilt angle
    if (angle > tiltThreshold) {
        // Positive tilt (forward movement)
        for (int i = 0; i < 5; i++) {
            if (angle > speedThresholds[i]) {
                speed = speedLevels[i];
            }
        }
    } else if (angle < -tiltThreshold) {
        // Negative tilt (backward movement)
        for (int i = 0; i < 5; i++) {
            if (angle < -speedThresholds[i]) {
                speed = speedLevels[i];
            }
        }
    } else {
        // Stop the motor if tilt angle is within the threshold
        speed = 0;
    }

    return speed;
}

// Function to get speed level index
int getSpeedLevel(int speed) {
    if (speed < speedLevels[0]) return 0;
    else if (speed < speedLevels[1]) return 1;
    else if (speed < speedLevels[2]) return 2;
    else if (speed < speedLevels[3]) return 3;
    else return 4;
}


