#include <NewPing.h>

#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping

// Ultrasonic sensor pins
#define TRIGGER_PIN_RIGHT 32
#define ECHO_PIN_RIGHT 33
#define TRIGGER_PIN_FRONT 25
#define ECHO_PIN_FRONT 26
#define TRIGGER_PIN_LEFT 27
#define ECHO_PIN_LEFT 14

// Motor driver pins for TB6612FNG
#define AIN1 12  // Motor A direction pin 1
#define AIN2 13  // Motor A direction pin 2
#define BIN1 18  // Motor B direction pin 1
#define BIN2 19  // Motor B direction pin 2
#define PWMA 4   // Motor A PWM pin
#define PWMB 5   // Motor B PWM pin
#define STBY 21  // Standby pin for TB6612FNG

NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);

int pwmChannelA = 0; // PWM channel for motor A
int pwmChannelB = 1; // PWM channel for motor B
int pwmFrequency = 20000; // PWM frequency in Hz
int pwmResolution = 8; // 8-bit PWM resolution (0-255)

// Timer variables to track how long the robot is stuck
unsigned long stuckTimer = 0;
unsigned long stuckTimeout = 1500; // 1.5 seconds threshold for being stuck

void setup() {
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Configure PWM channels
  ledcSetup(pwmChannelA, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelB, pwmFrequency, pwmResolution);

  // Attach PWM channels to pins
  ledcAttachPin(PWMA, pwmChannelA);
  ledcAttachPin(PWMB, pwmChannelB);

  // Set standby pin high to activate the motor driver
  digitalWrite(STBY, HIGH);
}

// Sensor distance variables
int distRight;
int distFront;
int distLeft;

// Thresholds and speed control
float rightWallThreshold = 7;  // Distance to keep from the right wall in cm
int frontThreshold = 9;     // Distance to detect obstacle in front in cm
int forwardSpeed = 180;      // Speed for motors (0-255)
int rightTurnThreshold = 20; // Minimum distance to turn right in cm

void loop() {
  // Read sensor values
  distRight = sonarRight.ping_cm();
  delay(10);
  distFront = sonarFront.ping_cm();
  delay(10);
  distLeft = sonarLeft.ping_cm();
  delay(10);

  Serial.print("Right: "); Serial.println(distRight);
  Serial.print("Left: "); Serial.println(distLeft);
  Serial.print("Front: "); Serial.println(distFront);

  // Main logic for right-wall following
  if (distFront >= frontThreshold) { // No obstacle in front
    stuckTimer = millis(); // Reset stuck timer
    if (distRight > rightTurnThreshold) { // Enough space on the right to turn
      turnLeft(); // Turn right if distance is greater than 20 cm
    } else if (distRight > rightWallThreshold) { // Too far from right wall
      slightLeft(); // Steer right
    } else if (distRight < rightWallThreshold) { // Too close to right wall
      slightRight(); // Steer left
    } else {
      moveForward(); // Move forward if distance is good
    }
  } else { // Obstacle in front
    stopMovement();
    if (millis() - stuckTimer > stuckTimeout) { // If stuck for too long
      backupAndRetry();
    } else { // Turn left if blocked on right and front
      if (distLeft > frontThreshold) { // Check left side
        turnLeft();  // If space is clear on the left, turn left
      } else {
        turnRight(); // If stuck with no space on the left, turn right
      }
      moveForwardAfterTurn();
    }
  }

  delay(100); // Small delay to stabilize the robot's movement
}

// Movement functions
void moveForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, forwardSpeed);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, forwardSpeed);
}

void slightLeft() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, forwardSpeed / 2); // Reduce left motor speed

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, forwardSpeed); // Right motor at full speed
}

void slightRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, forwardSpeed); // Left motor at full speed

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, forwardSpeed / 2); // Reduce right motor speed
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(pwmChannelA, forwardSpeed);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, forwardSpeed);

  delay(450); // Adjust for a sharp turn
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, forwardSpeed);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelB, forwardSpeed);

  delay(450); // Adjust for a sharp turn
}

void moveForwardAfterTurn() {
  moveForward();
  delay(50);  // Move forward for a short distance
}

void stopMovement() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(pwmChannelA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelB, 0);
}

// Backup and retry function when stuck
void backupAndRetry() {
  Serial.println("Backup and retry...");
  moveBackward();
  delay(300);
  slightLeft();  // Slight left to help adjust
  
  moveForwardAfterTurn();
  stuckTimer = millis(); // Reset stuck timer
}

void moveBackward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(pwmChannelA, forwardSpeed);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelB, forwardSpeed);
}
