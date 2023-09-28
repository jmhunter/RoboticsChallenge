/*

  Robotics Challenge - Version History

  v 1.0 - 15/03/14  - Backend code for the robotics - same code master - will be overwritten
  v 1.1 - 21/08/14  - Updated to include new challenges
  v 1.2 - 23/03/15  - Cleaned up and simplified
  v 1.3 - 10/05/16  - Updated to fix minor bugs.  Also has a TestAll function you use to
                      test all components are working
  v 1.4 - 20/09/23  - Complete re-write to support KeyeStudio robots now the orginal 4tronix
                      robots are no longer available

*/

// robo sets up all the functions for all robot movements and sensors
// Look in robo.h for all the #defines

// Inclusion of libraries for control of robot servos and sensors
#include <SR04.h>    // Ultrasonic Sensor library for KeyeStudio robots


// SR04 library for setup of Ultrasonic Sensor
SR04 sr04 = SR04(ULTRA_SONAR_TRIGGER, ULTRA_SONAR_ECHO);


// The setup routine runs once when you press reset
// This must be here to set your pins for the motor and do some initialisation
void setup()

{
// initialize the digital pins to be used as an output for the Motors
  pinMode(L1, OUTPUT);  // define PWM control pin of B motor as OUTPUT
  pinMode(L2, OUTPUT);  // define direction control pin of B motor as OUTPUT
  pinMode(L3, OUTPUT);  // define PWM control pin of A motor as OUTPUT
  pinMode(L4, OUTPUT);  // define direction control pin of A motor as OUTPUT

// initialize the digital pins to be used as an input for the Line Following Sensors
  pinMode(LINE_LEFT,  INPUT);  // define the pin of left line tracking sensor as INPUT
  pinMode(LINE_CENTRE,INPUT);  // define the pin of middle line tracking sensor as INPUT
  pinMode(LINE_RIGHT, INPUT);  // define the pin of right line tracking sensor as INPUT

// initialize the digital pins to be used as an output for the Matrix Display Screen
  pinMode(MATRIX_CLOCK,  OUTPUT);  // define the pin of matrix clock as OUTPUT
  pinMode(MATRIX_DISPLAY,OUTPUT);  // define the pin of matrix display to OUTPUT

// Start the Serial device and make sure the speed is set to 115,200 baud
  Serial.begin(115200);
  Serial.println("Starting Barclays Robot");

// Centre the Ultrasonic sensor servo as part of the robot set up routine
ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);  // Set the angle of Ultrasonic servo to 90 degrees
}


// robotMove routine provides the inputs to the motors for basic robot movement
void robotMove(int l1, int l2, int r1, int r2)
{
  analogWrite(L1, l1);
  digitalWrite(L2, l2);
  analogWrite(L3, r1);
  digitalWrite(L4, r2);
}


// forward basic movement routine
void forward(int wait,int vSpeedLeft,int vSpeedRight)

// ensures no value over 255 can be used for forward movement
{
  if ( vSpeedLeft > 255 )
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 )
    vSpeedRight = 255;

  Serial.println("Moving Forwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(vSpeedLeft, HIGH, vSpeedRight, HIGH);
  delay(wait);

//stop forward movement routine after the wait time
  halt(0);
}


// reverse basic movement routine
void reverse(int wait, int vSpeedLeft, int vSpeedRight)

// ensures no value over 255 can be used for reverse movement
{
  if ( vSpeedLeft > 255 )
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 )
    vSpeedRight = 255;

  Serial.println("Moving Backwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(vSpeedLeft, LOW, vSpeedRight, LOW);
  delay(wait);

//stop reverse movement routine after the wait time
  halt(0);
}


// turning right basic movement routine
void rightSpin(int wait, int vSpeed)

{
  Serial.println("Spinning right");
  robotMove(vSpeed, HIGH, vSpeed, LOW);
  delay(wait);

//stop turning right movement routine after the wait time
  halt(0);
}

// turning left basic movement routine
void leftSpin(int wait,int vSpeed)

{
  Serial.println("Spinning left");
  robotMove(vSpeed, LOW, vSpeed, HIGH);
  delay(wait);
  
//stop turning left movement routine after the wait time
  halt(0);
}


// stopping all basic movement routine
void halt(int wait)

{
  Serial.println("Stopping");
  robotMove(0, LOW, 0, LOW);
  delay(wait);
}


// LineSensor routines provide the inputs to the line sensors to detect dark lines on a lighter background

int readLineSensor(int pin)
{
  return digitalRead(pin);
}

// Takes the reading from the right line sensor and returns whether it is BLACK or WHITE
int rightLineSensor()
{
  int val = readLineSensor(LINE_RIGHT);
  Serial.println("Right Line Sensor: " + val);
  return val;
}

// Takes the reading from the left line sensor and returns whether it is BLACK or WHITE
int leftLineSensor()
{
  int val= readLineSensor(LINE_LEFT);
  Serial.println("Left Line Sensor: " + val);
  return val;
}

// Takes the reading from the centre line sensor and returns whether it is BLACK or WHITE
int centreLineSensor()
{
  int val= readLineSensor(LINE_CENTRE);
  Serial.println("Centre Line Sensor: " + val);
  return val;
}


// UltraSonic routines provide the inputs to the Ultrasonic sensor and pan motor for detection and movement of the Ultrasonic head

// Calculates the running angle of the Ultrasonic Pan Servo
void ultraServoPulse(int ULTRA_SERVO_PAN,int myangle)
{
  for(int i=0; i<30; i++)
  {
    int pulseWidth = (myangle*11)+500;
    digitalWrite(ULTRA_SERVO_PAN,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(ULTRA_SERVO_PAN,LOW);
    delay(20-pulseWidth/1000);
  }
}


// Takes the reading from the Ultrasonic sensor and returns whether an obstacle is detected ahead
boolean lookForward()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);         // re-centre the Ultrasonic pan servo to point forwards
  delay(ULTRA_SONAR_WAIT);
  ULTRA_SONAR_DISTANCE=sr04.Distance();                        // obtain the distance value detected by ultrasonic sensor
  if((ULTRA_SONAR_DISTANCE < 20)&&(ULTRA_SONAR_DISTANCE > 0))  // if the distance is greater than 0 and less than 20 then...
  {
    Serial.println("Obstacle Not Detected Ahead");
    return false;
  }
  Serial.println("Obstacle Detected Ahead");
  return true;
}


// Takes the reading from the Ultrasonic sensor and returns whether an obstacle is detected to the left
boolean lookLeft()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_LEFT);       // turn the Ultrasonic pan servo left
  delay(ULTRA_SONAR_WAIT);
  ULTRA_SONAR_DISTANCE=sr04.Distance();                        // obtain the distance value detected by ultrasonic sensor
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);         // re-centre the Ultrasonic pan servo to point forwards

  if((ULTRA_SONAR_DISTANCE < 20)&&(ULTRA_SONAR_DISTANCE > 0))  // if the distance is greater than 0 and less than 20 then...
  {
    Serial.println("Obstacle Not Detected On The Left");
    return false;
  }
  Serial.println("Obstacle Detected On The Left");
  return true;
}


// Takes the reading from the Ultrasonic sensor and returns whether an obstacle is detected to the right
boolean lookRight()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_RIGHT);      // turn the Ultrasonic pan servo right
  delay(ULTRA_SONAR_WAIT);
  ULTRA_SONAR_DISTANCE=sr04.Distance();                        // obtain the value detected by ultrasonic sensor
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);         // re-centre the Ultrasonic pan servo to point forwards

  if((ULTRA_SONAR_DISTANCE < 20)&&(ULTRA_SONAR_DISTANCE > 0))  // if the distance is greater than 0 and less than 20 then...
  {
    Serial.println("Obstacle Not Detected On The Right");
    return false;
  }
  Serial.println("Obstacle Detected On The Right");
  return true;
}


/*int Ultrasonic()
{

  int cm=0;
  unsigned int pingTime=0;
  Serial.print("Sonar Ping: ");

  for ( int i=0; i < ULTRA_SONAR_SAMPLE_SIZE; i++)
  {
    pingTime = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    delay(ULTRA_SONAR_WAIT);                      // Wait between pings, 20ms should be the shortest delay between pings.
    cm += pingTime / US_ROUNDTRIP_CM;
  }

  cm /= ULTRA_SONAR_SAMPLE_SIZE;

  if ( cm == 0 || cm > ULTRA_SONAR_MAX_RANGE)
    cm = ULTRA_SONAR_MAX_RANGE;  // Clear!

  Serial.print(cm);
  Serial.println("cm");

  return cm;
}


void pointLeft()
{
  panServo.attach(ULTRA_SERVO_PAN);
  panServo.write(ULTRA_SERVO_PAN_LEFT);
  Serial.println("Point Left");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

void pointRight()
{
  panServo.attach(ULTRA_SERVO_PAN);
  panServo.write(ULTRA_SERVO_PAN_RIGHT);
  Serial.println("Point Right");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

void pointCentre()
{
  servopulse(servopin, ULTRA_SERVO_CENTRE);  // The angle of servo is set to point directly forwards
//  panServo.attach(ULTRA_SONAR_PAN);
//  panServo.write(ULTRA_SERVO_CENTRE);
  Serial.println("Point Centre");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
//  panServo.detach();
}

void pointValue(int pos)
{
  panServo.attach(ULTRA_SERVO_PAN);
  Serial.print("Point Value: ");
  Serial.println(pos);
  panServo.write(pos);
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

*/
