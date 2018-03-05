/*

  V 1.0 - 15/03/14  - Backend code for the robotics - same code master - will be overwritten
  V 1.1 - 21/08/14  - Updated to include new challenges
  V 1.2 - 23/03/15  - Cleaned up and simplified
  V 1.3 - 10/05/16  - Updated to fix minor bugs
  V 1.4 - 06/03/18  - JMH removed debugging serial prints and test code, they are never used
  
  Robotics Challenge - - all supporting code

*/

//Look in robo.h for all the #defines

// New ping stuff - wont need library as the CPP is included in sketch
#include <NewPing.h>
#include <Servo.h> // need the standard Arduino servo library

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_PING_RANGE);

// Pan and tilt servo objects
Servo panServo, tiltServo;

// The setup routine runs once when you press reset:
// This must be here to set your pins for the motor and do some initialisation
void setup()
{                
  // initialize the digital pins we will use as an output for the Motors
  pinMode(L1, OUTPUT);     
  pinMode(L2, OUTPUT);     
  pinMode(L3, OUTPUT);     
  pinMode(L4, OUTPUT);
      
  Serial.begin(9600);  // Start the Serial
  Serial.println("Starting Barclays Robot");
  // We don't print debugging information within robo.ino any longer, but
  // Serial.println() can still be used within user sketches
  
  tiltCentre();
  pointCentre();
  
}

int readLineSensor(int pin)
{
  int sensorSignal = 0;
  
  for ( int i=0; i < SAMPLE_SIZE_LINE; i++ )
  {
    sensorSignal += analogRead(pin);// Read value from sensor  
  }
   
  sensorSignal /= SAMPLE_SIZE_LINE;
  
  //Always returns 0 or 1, but whether that means BLACK or WHITE can be changed in robo.h depending on the sensor version.
  if ( sensorSignal < LINE_THRESHOLD )
    return 0;
    
  return 1;  
}


int rightLineSensor()
{
  int val = readLineSensor(RIGHT_LINE);
  return val;
}

int leftLineSensor()
{
  int val= readLineSensor(LEFT_LINE);
  return val;
}

// robMove routine switches the correct inputs to the L298N for the direction selected.
void robMove(int l1, int l2, int r1, int r2)
{
  analogWrite(L1, l1);
  digitalWrite(L2, l2);
  analogWrite(L3, r1);
  digitalWrite(L4, r2);  
}

void reverse(int wait, int vSpeedLeft, int vSpeedRight)
{
  if ( vSpeedLeft > 255 ) 
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 ) 
    vSpeedRight = 255;

  robMove(255-vSpeedLeft, HIGH, 255-vSpeedRight, HIGH);    // when reversing, the speed needs to be opposite, so subtract from 255
  delay(wait);
  
  //Amended to stop after the wait time
  halt(0);
}

void forward(int wait,int vSpeedLeft,int vSpeedRight)
{
  if ( vSpeedLeft > 255 ) 
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 ) 
    vSpeedRight = 255;

  robMove(vSpeedLeft, LOW, vSpeedRight, LOW);
  delay(wait);
  
  //Amended to stop after the wait time
  halt(0);
}

void rightSpin(int wait, int vSpeed)
{
  robMove(vSpeed, LOW, 255 - vSpeed, HIGH);
  delay(wait);
  
  //Amended to stop after the wait time
  halt(0);
}

void leftSpin(int wait,int vSpeed)
{
  robMove(255 - vSpeed, HIGH, vSpeed, LOW);
  delay(wait);
  
  //Amended to stop after the wait time
  halt(0);
}


void halt(int wait)
{
  robMove(0, LOW, 0, LOW);
  delay(wait);
}


boolean leftObstacleSensor()
{
 
  if ( digitalRead(LEFT_OBSTACLE) == 0 ) // Test left sensor
  {
    return true;
  }
  
  return false;
}

boolean rightObstacleSensor()
{
 
  if ( digitalRead(RIGHT_OBSTACLE) == 0 ) // Test right sensor
  {
    return true;
  }
  
  return false;
}


int Ultrasonic()
{
 
  int cm=0;
  unsigned int pingTime=0;
  
  for ( int i=0; i < SAMPLE_SIZE_ULTRA; i++)
  {
    pingTime = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    delay(SONAR_WAIT);                      // Wait between pings, 20ms should be the shortest delay between pings.
    cm += pingTime / US_ROUNDTRIP_CM;
  }
  
  cm /= SAMPLE_SIZE_ULTRA;
  
  if ( cm == 0 || cm > MAX_PING_RANGE) 
    cm = MAX_PING_RANGE;  // Clear!

  return cm;
}


void pointLeft()
{
  panServo.attach(PAN_PIN); 
  panServo.write(SERVO_LEFT);
  delay(SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

void pointRight()
{
  panServo.attach(PAN_PIN); 
  panServo.write(SERVO_RIGHT);
  delay(SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

void pointCentre()
{
  panServo.attach(PAN_PIN); 
  panServo.write(SERVO_CENTRE);
  delay(SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}

void pointValue(int pos)
{
  panServo.attach(PAN_PIN);
  panServo.write(pos);
  delay(SERVO_WAIT); // wait for servo to get there
  panServo.detach();
}


void tiltUp()
{
  tiltServo.attach(TILT_PIN);
  tiltServo.write(SERVO_UP);
  delay(SERVO_WAIT); // wait for servo to get there
  tiltServo.detach();
}

void tiltDown()
{
  tiltServo.attach(TILT_PIN);
  tiltServo.write(SERVO_DOWN);
  delay(SERVO_WAIT); // wait for servo to get there
  tiltServo.detach();
}

void tiltCentre()
{
  tiltServo.attach(TILT_PIN);
  tiltServo.write(SERVO_CENTRE);
  delay(SERVO_WAIT); // wait for servo to get there
  tiltServo.detach();
}

void tiltValue(int pos)
{
  tiltServo.attach(TILT_PIN);
  tiltServo.write(pos);
  delay(SERVO_WAIT); // wait for servo to get there
  tiltServo.detach();
}

