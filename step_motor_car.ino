#include <AccelStepper.h>

// configuration for step motor
#define HALFSTEP 8
#define FULLSTEP 4
#define MOTOR_SPEED 500
#define SAFEDISTANCE 25

// define direction for each wheel
#define LEFT_FORWARD  (-MOTOR_SPEED)
#define LEFT_BACKWARD (MOTOR_SPEED)

#define RIGHT_FORWARD (MOTOR_SPEED)
#define RIGHT_BACKWARD (-MOTOR_SPEED)

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper wheelRight(FULLSTEP, 8, 10, 9, 11); // right wheel -- please note here: NOT 8  9 10 11  it's 8 10 9 11  this confused me a lot  :)  if wrong pin sequence, the motor will not work properly.
AccelStepper wheelLeft(FULLSTEP, 4, 6, 5, 7);   // left wheel   -- please note here: NOT 4  5  6  7  it's 4 6  5  7

// distance detecting interval
unsigned long lastMeasureTime = 0;
unsigned long measureInterval = 500;  //ms

// sensor struct and constructor
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  float samples[3];
  int sampleIndex;
  unsigned long lastSampleTime;
  float filteredDistance;
  bool isReady;

  UltrasonicSensor(int trig, int echo)
    : trigPin(trig), echoPin(echo), sampleIndex(0),
      lastSampleTime(0), filteredDistance(9999.0), isReady(false) {}
};

#define LEFT_ECHO 14  // A1 = 15
#define LEFT_TRIG 15  // A0 = 14

// define each distance sensor
UltrasonicSensor frontSensor(3, 2);     // front sensor, trig, echo
UltrasonicSensor backSensor(13, 12);    // back sensor,  trig, echo
UltrasonicSensor leftSensor(A1, A0);    // left sensor,  trig, echo
UltrasonicSensor rightSensor(A3, A2);   // right sensor, trig, echo

void setup() {

  Serial.begin(9600);

  // initialize step motor
  wheelRight.setMaxSpeed(2500.0);      
  wheelRight.setAcceleration(100.0);
  wheelRight.setSpeed(LEFT_FORWARD);

  wheelLeft.setMaxSpeed(2500.0);
  wheelLeft.setAcceleration(100.0);
  wheelLeft.setSpeed(RIGHT_FORWARD);

  // intitialize sensors
  pinMode(frontSensor.trigPin, OUTPUT);
  pinMode(frontSensor.echoPin, INPUT);

  pinMode(backSensor.trigPin, OUTPUT);
  pinMode(backSensor.echoPin, INPUT);

  pinMode(leftSensor.trigPin, OUTPUT);
  pinMode(leftSensor.echoPin, INPUT);

  pinMode(rightSensor.trigPin, OUTPUT);
  pinMode(rightSensor.echoPin, INPUT);

  Serial.println("Ultrasonic avoidance + stepper motor running...");
}

void loop() {
    // keep step motor running
    wheelRight.runSpeed();
    wheelLeft.runSpeed();

    unsigned long now = millis();
    if (now - lastMeasureTime >= measureInterval) {
      lastMeasureTime = now;
      measureInterval = 500;


      // measure distance of each direction
      updateSensor(frontSensor);
      updateSensor(backSensor);
      updateSensor(leftSensor);
      updateSensor(rightSensor);

      float dFront = 0;
      float dBack = 0;
      float dLeft = 0;
      float dRight = 0;

      // ---------- front ----------
      if (frontSensor.isReady) {
        frontSensor.isReady = false;
        dFront = frontSensor.filteredDistance;
        Serial.println("Front: " + String(dFront));
      }

      // ---------- back ----------
      if (backSensor.isReady) {
        backSensor.isReady = false;
        dBack = backSensor.filteredDistance;
        Serial.println("Back: " + String(dBack));
      }

      // ---------- left ----------
      if (leftSensor.isReady) {
        leftSensor.isReady = false;
        dLeft = leftSensor.filteredDistance;
        Serial.println("Left: " + String(dLeft));
      }

      // ---------- right ----------
      if (rightSensor.isReady) {
        rightSensor.isReady = false;
        dRight = rightSensor.filteredDistance;
        Serial.println("Right: " + String(dRight));
      }


      if (dFront <= SAFEDISTANCE && 
          dBack  <= SAFEDISTANCE && 
          dLeft  <= SAFEDISTANCE && 
          dRight <= SAFEDISTANCE)
      {
          stopMotors();
          Serial.println("Blocked → stopMotors");      
      }
      else if (dFront <= SAFEDISTANCE)
      {
        if(dRight >= SAFEDISTANCE)
        {
          turnRightQuick();
          Serial.println("Front: " + String(dFront) + " cm -> Front blocked → turnRightQuick");      
        }
        else if(dLeft >= SAFEDISTANCE)
        {
          turnLeftQuick();
          Serial.println("Front: " + String(dFront) + " cm -> Front blocked → turnLeftQuick");
        }
        else
        {
          goBackward();
          Serial.println("Front: " + String(dFront) + " cm -> Front blocked → Go Backward");      
        }
      }
      else if (dBack <= SAFEDISTANCE) 
      {
        goForward();
        Serial.println("Back: " + String(dBack) + " cm -> Back blocked → GO FORWARD");      
      }
      else if (dLeft <= SAFEDISTANCE) 
      {
        turnRight();
        Serial.println("Left " + String(dLeft) + " Left blocked → turn right");
      }
      else if (dRight <= SAFEDISTANCE) 
      {
        turnLeft();
        Serial.println("Right " + String(dRight) + " Right blocked → turn left");
      }
      else {
        goForward();
        Serial.println("No Blocks → GO FORWARD");      
      }
    }
}



/////////////////////////////////////////////
FUNCTIONS
/////////////////////////////////////////////

void goForward() {
  wheelRight.setSpeed(LEFT_FORWARD);
  wheelLeft.setSpeed(RIGHT_FORWARD);
}

void goBackward() {
  wheelRight.setSpeed(LEFT_BACKWARD);
  wheelLeft.setSpeed(RIGHT_BACKWARD);
}

void turnLeft() {
  wheelRight.setSpeed(RIGHT_BACKWARD);
  wheelLeft.setSpeed(RIGHT_BACKWARD);
}

void turnRight() {
  wheelRight.setSpeed(LEFT_BACKWARD);
  wheelLeft.setSpeed(LEFT_BACKWARD);
}

void turnLeftQuick() {
  wheelRight.setSpeed(RIGHT_BACKWARD);
  wheelLeft.setSpeed(RIGHT_BACKWARD);
  measureInterval = 4000; // modify the measureInterval tempoerarily to turn longer time.
}

void turnRightQuick() {
  wheelRight.setSpeed(LEFT_BACKWARD);
  wheelLeft.setSpeed(LEFT_BACKWARD);
  measureInterval = 4000; // modify the measureInterval tempoerarily to turn longer time.
}

void stopMotors() {
  wheelRight.setSpeed(0);
  wheelLeft.setSpeed(0);
}

float measureDistanceOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 10000UL);
  float dist = (duration * 0.034) / 2.0;
  if (dist <= 1) {
      dist = 9999.0;
  }
  return dist;
}


void updateSensor(UltrasonicSensor &sensor) {
  const unsigned long SAMPLE_INTERVAL = 10;

  if (millis() - sensor.lastSampleTime >= SAMPLE_INTERVAL) {
    sensor.lastSampleTime = millis();

    float dist = measureDistanceOnce(sensor.trigPin, sensor.echoPin);

    sensor.filteredDistance = dist;

    sensor.isReady = true;
  }
}


