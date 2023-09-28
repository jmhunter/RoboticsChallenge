/*

  Robotics Challenge - Version History

  v 1.0 - 15/03/14  - Backend code for the robotics - same code master - will be overwritten
  v 1.1 - 21/08/14  - Updated to include new challenges
  v 1.2 - 23/03/15  - Cleaned up and simplified
  v 1.3 - 10/05/16  - Updated to fix minor bugs.  Also has a TestAll function you use to
                      test all components are working
  v 1.4 - 01/10/23  - Complete re-write to standardise the 4tronix code and variables with
                    - the ones used by the KeyeStudio robots now the oringal 4tornix robots
                    - are no longer avialable.

*/

// robo sets up all the functions for all robot movements and sensors
// Look in robo.h for all the #defines

// Inclusion of libraries for control of robot servos and sensors
#include <NewPing.h> // Ultrasonic Sensor library for 4tronix robots
#include <Servo.h>   // Ultrasonic Servo library for 4tronix robots

// NewPing library for setup of Ultraonic sensor pins and maximum distance
NewPing sonar(ULTRA_SONAR_TRIGGER, ULTRA_SONAR_ECHO, ULTRA_SONAR_MAX_RANGE);

// Servo library for setup of Ultrasonic sensor pan and tilt servo
Servo ultraPanServo, ultraTiltServo;


// The setup routine runs once when you press reset
// This must be here to set your pins for the motor and do some initialisation
void setup()

{
// initialize the digital pins to be used as an output for the Motors
  pinMode(L1, OUTPUT);  // define PWM control pin of B motor as OUTPUT
  pinMode(L2, OUTPUT);  // define direction control pin of B motor as OUTPUT
  pinMode(L3, OUTPUT);  // define PWM control pin of A motor as OUTPUT
  pinMode(L4, OUTPUT);  // define direction control pin of A motor as OUTPUT

// Start the Serial device and make sure the speed is set to 115,200 baud
  Serial.begin(115200);
  Serial.println("Starting Barclays Robot");

// Centre the Ultrasonic sensor pan and tilt servos as part of the robot set up routine
  tiltCentre();
  pointCentre();
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
  robotMove(vSpeedLeft, LOW, vSpeedRight, LOW);
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
// when reversing, the speed needs to be opposite, so subtract speed from 255
  robotMove(255-vSpeedLeft, HIGH, 255-vSpeedRight, HIGH);
  delay(wait);

//stop reverse movement routine after the wait time
  halt(0);
}


// turning right basic movement routine
void rightSpin(int wait, int vSpeed)

{
  Serial.println("Spinning right");
// for the motor on the right, the speed needs to be opposite, so subtract speed from 255
  robotMove(vSpeed, LOW, 255 - vSpeed, HIGH);
  delay(wait);
  
//stop turning right movement routine after the wait time
  halt(0);
}


// turning left basic movement routine
void leftSpin(int wait,int vSpeed)

{
  Serial.println("Spinning left");
// for the motor on the left, the speed needs to be opposite, so subtract speed from 255
  robotMove(255 - vSpeed, HIGH, vSpeed, LOW);
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
  int sensorSignal = 0;
  
  for ( int i=0; i < LINE_SAMPLE_SIZE; i++ )
  {
    sensorSignal += analogRead(pin);  // Read value from sensor  
  }
   
  sensorSignal /= LINE_SAMPLE_SIZE;
  
// Sensor always returns 0 or 1, but whether that means BLACK or WHITE can be changed in robo.h depending on the sensor version
  if ( sensorSignal < LINE_THRESHOLD )
    return 0;
    
  return 1;  
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


// ObstacleSensor routines provide the inputs to the infrared obstacle sensors to detect the presence of an object
boolean leftObstacleSensor()
{

// Takes the reading from the left obstcle sensor and returns an object is detected
  if ( digitalRead(OBSTACLE_LEFT) == 0 )
  {
    Serial.println("Left Obstacle Detected");
    return true;
  }

  return false;
}

boolean rightObstacleSensor()
{

// Takes the reading from the right obstcle sensor and returns an object is detected
  if ( digitalRead(OBSTACLE_RIGHT) == 0 )
  {
    Serial.println("Right Obstacle Detected");
    return true;
  }

  return false;
}


// UltraSonic routines provide the inputs to the Ultrasonic sensor and pan motor for detection and movement of the Ultrasonic head
int Ultrasonic()
{
 
  int cm=0;
  unsigned int pingTime=0;
  Serial.print("Sonar Ping: ");  
  
  for ( int i=0; i < ULTRA_SONAR_SAMPLE_SIZE; i++)
  {
    pingTime = sonar.ping();  // Send ping, get ping time in microseconds (uS).
    delay(ULTRA_SONAR_WAIT);  // Wait between pings, 20ms should be the shortest delay between pings.
    cm += pingTime / US_ROUNDTRIP_CM;
  }
  
  cm /= ULTRA_SONAR_SAMPLE_SIZE;
  
  if ( cm == 0 || cm > ULTRA_SONAR_MAX_RANGE) 
    cm = ULTRA_SONAR_MAX_RANGE;  // Clear!

  Serial.print(cm);
  Serial.println("cm");
  
  return cm;
}


// Ultrasonic routines provide the inputs to the turn Ultrasonic servo motors to detect the presence of an object

// Ultrasonic pan servo centering routine
void pointCentre()
{
  ultraPanServo.attach(ULTRA_SERVO_PAN);
  ultraPanServo.write(ULTRA_SERVO_CENTRE);
  Serial.println("Point Centre");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraPanServo.detach();
}


// Ultrasonic pan servo pointing left routine
void pointLeft()
{
  ultraPanServo.attach(ULTRA_SERVO_PAN);
  ultraPanServo.write(ULTRA_SERVO_PAN_LEFT);
  Serial.println("Point Left");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraPanServo.detach();
}


// Ultrasonic pan servo pointing right routine
void pointRight()
{
  ultraPanServo.attach(ULTRA_SERVO_PAN);
  ultraPanServo.write(ULTRA_SERVO_PAN_RIGHT);
  Serial.println("Point Right");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraPanServo.detach();
}


// Ultrasonic pan servo pointing to a specific value routine
void pointValue(int pos)
{
  ultraPanServo.attach(ULTRA_SERVO_PAN);
  Serial.print("Point Value: ");
  Serial.println(pos);
  ultraPanServo.write(pos);
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraPanServo.detach();
}


// Ultrasonic tilt servo centering routine
void tiltCentre()
{
  ultraTiltServo.attach(ULTRA_SERVO_TILT);
  ultraTiltServo.write(ULTRA_SERVO_CENTRE);
  Serial.println("Tilt Centre");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraTiltServo.detach();
}


// Ultrasonic tilt servo pointing up routine
void tiltUp()
{
  ultraTiltServo.attach(ULTRA_SERVO_TILT);
  ultraTiltServo.write(ULTRA_SERVO_TILT_UP);
  Serial.println("Tilt Up");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraTiltServo.detach();
}


// Ultrasonic tilt servo pointing down routine
void tiltDown()
{
  ultraTiltServo.attach(ULTRA_SERVO_TILT);
  ultraTiltServo.write(ULTRA_SERVO_TILT_DOWN);
  Serial.println("Tilt Down");
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraTiltServo.detach();
}


// Ultrasonic tilt servo pointing to a specific position routine
void tiltValue(int pos)
{
  ultraTiltServo.attach(ULTRA_SERVO_TILT);
  Serial.print("Tilt Value: ");
  Serial.println(pos);
  ultraTiltServo.write(pos);
  delay(ULTRA_SERVO_WAIT); // wait for servo to get there
  ultraTiltServo.detach();
}

/*
void testAll()
{
  long startTime;  // Captures startTime in clock time - time is milliseconds between them
  long endTime;
  
  // ROBOTEERS this is the test all function - setup() will execute this once or it needs to be called from loop()
  
  Serial.println("TESTING MOVEMENT SHOULD TAKE ABOUT 10 seconds");
  Serial.println("FULL POWER 255");
  forward(1000,255,255);
  halt(1000);
  
  reverse(1000,255,255);
  halt(1000);
 
  Serial.println("HALF POWER 180");
  forward(1000,180,180);
  halt(1000);
  
  reverse(1000,180,180);
  halt(1000);
  
  
 Serial.println("NOW TESTING PAN AND TILT 5 TIMES");
 Serial.println("TESTING PAN 5 TIMES - PLEASE ENSURE THIS IS A PAN ACTION");
 
  for ( int i=0;i<5;i++)
  {
    pointCentre();
    delay(500);
    pointLeft();
    delay(500);
    pointRight();
    delay(500);
    pointCentre();
    
    delay(1000);
  }
  
 Serial.println("TESTING TILT 5 TIMES - PLEASE ENSURE THIS IS A TILT ACTION");
 
  for ( int i=0;i<5;i++)
  {
    tiltCentre();
    delay(500);
    tiltUp();
    delay(500);
    tiltDown();
    delay(500);
    tiltCentre();
    
    delay(1000);
  }
 
  
  Serial.println("NOW TESTING OBSTACLE SENSORS FOR 30 SECONDS ");
  Serial.println("TEST SENSOR NOW PLEASE - TILT and PAN SENSOR WILL MOVE");

  startTime = millis();
  endTime = millis();
  
  while(endTime-startTime < 30000)
  {
    
    if ( leftObstacleSensor() == true )
    {
       pointLeft();
       tiltUp();
    }
  
    if ( rightObstacleSensor() == true )
    {
       pointRight();
       tiltDown();
    }
    
    endTime = millis();
  }
  
  Serial.println("NOW TESTING ULTRASONIC SENSORS FOR 30 SECONDS ");
  Serial.println("TEST SENSOR BY PLACING YOUR HAND OR OBJECT IN FRONT OF ULTRASONIC ( EYES)");
  Serial.println("IF THE DISTANCE IS LESS THAN 90CM IT SHOULD PRINT");

  startTime = millis();
  endTime = millis();
  
  while(endTime-startTime < 30000)
  {

    int cm =  Ultrasonic();
    
    if ( cm < 90 ) {
      Serial.print("Distance is: ");
      Serial.println(cm);
    }
    
    endTime = millis();
  }
  
    pointCentre();
    tiltCentre();
  
  Serial.println("NOW TESTING LINE FOLLOW SENSORS FOR 30 SECONDS ");
  Serial.println("TEST SENSOR NOW PLEASE USING BLACK or WHITE for EACH SENSOR");

  startTime = millis();
  endTime = millis();
  
  while(endTime-startTime < 30000)
  {
    
    if ( leftLineSensor() == BLACK ) 
      Serial.println("LEFT SENSOR READS BLACK");
   
   
    if ( leftLineSensor() == WHITE )    
     Serial.println("LEFT SENSOR READS WHITE");
    
    if ( rightLineSensor() == BLACK ) 
      Serial.println("RIGHT SENSOR READS BLACK");
   
   
    if ( rightLineSensor() == WHITE )    
     Serial.println("RIGHT SENSOR READS WHITE");
    
    endTime = millis();
  }
   
}
*/
