// arduino-obstacle-avoiding-robot.ino
// Arduino obstacle avoiding robot using ultra-sonic sensor
// 
// Making an obstacle avoiding robot using Arduino
// http://myoungjinkim.github.io
//

#include <NewPing.h>
#include <Servo.h>

// Ultra-sonic sensor pins
#define SONAR_TRIG 10
#define SONAR_ECHO 11

// minimum distance HC-SR04 can detect
#define SONAR_MIN_DISTANCE 3

// maximum distance to detect an obstacle. Though HC-SR04 can detect 
// up to 4 meters, we only detect an obstance within 2 meters.
#define SONAR_MAX_DISTANCE 200

// Since HC-SR04 operates on 50Hz, we delay 25ms between each detection.
#define SONAR_DELAY 25

// Ultra-sonic sensor object
NewPing sonar(SONAR_TRIG, SONAR_ECHO, SONAR_MAX_DISTANCE);

// Servo values
#define SERVO_PWM 9     // servo motor data pin
#define SERVO_FRONT 90  // front angle
#define SERVO_LEFT 130  // left angle
#define SERVO_RIGHT 50  // right angle
#define SERVO_DELAY 700 // delay between angle change

// Servo motor object
Servo servo;

// L298N pins
#define IN1 3
#define IN2 4
#define ENA 5

#define IN3 8
#define IN4 7
#define ENB 6

// Motor values

// The minimum value of PWM to drive an DC motor.
// The value should be set according to the motor characteristics
// of a vehicle. In my system, motor A works above 65 whereas 
// motor B moves above 77. To drive the car as straight as 
// possible, I make motors stop under 80.
#define MOTOR_MIN_PWM 80
#define MOTOR_MAX_PWM 255

#define MOTOR_DRIVE_PWM 100   // motor drive pwm
#define MOTOR_TURN_DELAY 400  // delay for each turn

/*
 * Drives two motors according to the given pwm values.
 * 
 * To avoid write several functions for each motor direction, I simply
 * drive motor forward if the given value is positive and backward if
 * negative. To stop a motor, just give 0.
 * 
 * Since every DC motors have differenct characteristics, I stop a 
 * motor if the absolute value of the given pwm value is less than 
 * MOTOR_MIN_PWM to drive the vehicle as strait as possible.
 * 
 * @param pwmA PWM value of motor A. An integer value between -255 and 255.
 * @param pwmB PWM value of motor B. An integer value between -255 and 255.
 */
void drive(int pwmA, int pwmB) {
  // constrain motor speed between -255 and 255 
  pwmA = constrain(pwmA, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  pwmB = constrain(pwmB, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);

  // motor A direction
  if (pwmA > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (pwmA < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // motor B direction
  if (pwmB > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (pwmB < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // stop motors if the absolute value of pwm is less than MOTOR_MIN_PWM
  if (abs(pwmA) < MOTOR_MIN_PWM) pwmA = 0;
  if (abs(pwmB) < MOTOR_MIN_PWM) pwmB = 0;

  // pwm for each motor.
  analogWrite(ENA, abs(pwmA));
  analogWrite(ENB, abs(pwmB));
}


void setup() {
  Serial.begin(9600);

  // set all the motor pins OUTPUT mode
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // setup servo motor
  servo.attach(SERVO_PWM);
  servo.write(SERVO_FRONT);
  delay(SERVO_DELAY);
}

#define AVOIDING_DISTANCE 20

void loop() {
  // get the distance to an obstacle
  int distance = getDistance();
  if (distance < AVOIDING_DISTANCE) {
    // stop
    drive(0, 0);
    // look left
    servo.write(SERVO_LEFT);
    delay(SERVO_DELAY);
    int leftDistance = getDistance();
    delay(SONAR_DELAY);
    // look right
    servo.write(SERVO_RIGHT);
    delay(SERVO_DELAY);
    int rightDistance = getDistance();
    delay(SONAR_DELAY);
    // make servo face front
    servo.write(SERVO_FRONT);
    delay(SERVO_DELAY);
    
    // turn according to the distance to obstacles
    if (leftDistance < distance && rightDistance < distance) {
      drive(MOTOR_DRIVE_PWM, -MOTOR_DRIVE_PWM);
      delay(1000);
    } else if (leftDistance < rightDistance) {
      // turn right
      drive(-MOTOR_DRIVE_PWM, MOTOR_DRIVE_PWM);
      delay(MOTOR_TURN_DELAY);
    } else {
      // turn left
      drive(MOTOR_DRIVE_PWM, -MOTOR_DRIVE_PWM);
      delay(MOTOR_TURN_DELAY);
    }
    drive(0, 0);
  } else {
    // delay for ultra-sonic sensor
    delay(SONAR_DELAY);
    // no obstacle in front of the robot! go forward!
    drive(MOTOR_DRIVE_PWM, MOTOR_DRIVE_PWM);
  }
}

/*
 * Returns the distance to an obstacle in front of sonar sensor.
 * Notice that the sensor we use returns 0 if there is no obstacle 
 * in front of it. Since this leads to lots of compare compications,
 * it returns the double of maximum distance if the sensor value is 0.
 * 
 * @return the distance to an obstacle in front of sonar sensor.
 */
int getDistance() {
  int distance = sonar.ping_cm();
  if (distance < SONAR_MIN_DISTANCE) {
    distance = SONAR_MAX_DISTANCE * 2;
  }
  return distance;
}

