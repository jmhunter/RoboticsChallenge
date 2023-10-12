#include"robo.h"
/* 
This is a test suite, to make sure your KeyeStudio robot is working correctly.
                                        ----------
                                        
*** THIS SKETCH NEEDS UPDATING BECAUSE THE KEYESTUDIO ROBOT HAS NO OBSTACLE SENSORS ***
*** ALSO THE TEST SCRIPT FROM THE MOVE SKETCH HAS BEEN MOVED HERE ***

To use:
 * Hold your hand over the line follower sensors.
 * The line follower sensors will move the robot's head left and right, or move the robot.
 
If both left and right obstacle sensors are triggered together, the robot will move forwards, then backwards, then left, then right.
If the left and centre line follower sensors are triggered together, the robot's head will look to the left.
If the right and centre line follower sensors are triggered together, the robot's head will look to the right.

The ultrasonic sensor is not currently included in this test suite.
*/

void loop()
{

  if (leftLineSensor() == WHITE && centreLineSensor() == BLACK && rightLineSensor() == WHITE )
  {
    forward(150,100,100);
    delay(2000);
    forward(1000,150,150);
    reverse(1000,150,150);
    leftSpin(1000,150);
    rightSpin(1000,150);
    return;
  }

  if (leftLineSensor() == WHITE && centreLineSensor() == WHITE && rightLineSensor() == BLACK )
  {
    pointLeft();
    delay(2000);
    pointCentre();
    return;
  }

  if (leftLineSensor() == BLACK && centreLineSensor() == WHITE && rightLineSensor() == WHITE )
  {
    pointRight();
    delay(2000);
    pointCentre();
    return;
  } 

}
