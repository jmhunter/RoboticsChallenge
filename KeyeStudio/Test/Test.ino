/* 
 * This is a test suite, to make sure your robot is working correctly.
 * 
 * To use:
 * 
 * Hold your hand in front of the obstacle sensors. The robot's head
 * should point left when the left obstacle sensor is triggered, and
 * right when the right obstacle sensor is triggered.
 * 
 * The line follower sensors will move the robot's head up and down.
 * 
 * If both obstacle sensors are triggered together, the robot will
 * move forwards; if both line follower sensors are triggered together,
 * the robot will move backwards.
 * 
 * The ultrasonic sensor is not currently included in this test suite.
 * 
 */

// Uncomment these lines to run the testing script
  //while(1)  // Use this to repeat forever
  // testAll();
  
#include "robo.h"
// Define some variables to track state changes
bool prevleftLineSensor, prevleftObstacleSensor, prevrightLineSensor, prevrightObstacleSensor;

// Template for each sensor
#define HANDLESENSOR(sensor, actionTrue, actionFalse) \
  if (prev##sensor != sensor()) { \
    if (prev##sensor==false) { actionTrue; } else { actionFalse; } \
  } \
  prev##sensor=sensor();

void loop()
{
  // Obstacle sensors make the robot's head point left and right
  HANDLESENSOR(leftObstacleSensor, pointLeft(), pointCentre())
  HANDLESENSOR(rightObstacleSensor, pointRight(), pointCentre())

  // Line follower sensors make the robot's head move up and down
  HANDLESENSOR(leftLineSensor, tiltCentre(), tiltUp())
  HANDLESENSOR(rightLineSensor, tiltCentre(), tiltDown())

  // Are both the (obstacle|line) sensors active? If so, run the motors
  if (leftObstacleSensor()==true && rightObstacleSensor()==true)
    forward(500, 255, 255);
  if (leftLineSensor()==false && rightLineSensor()==false)
    reverse(500, 255, 255);
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
  
  
  Serial.println("NOW TESTING OBSTACLE SENSORS FOR 30 SECONDS ");
  Serial.println("TEST SENSOR NOW PLEASE - TILT and PAN SENSOR WILL MOVE");

  startTime = millis();
  endTime = millis();
  
  while(endTime-startTime < 30000)
  {
    
    if ( leftObstacleSensor() == true )
    {
       pointLeft();
    }
  
    if ( rightObstacleSensor() == true )
    {
       pointRight();
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
