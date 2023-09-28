/* 
This is a test suite, to make sure your 4tronix robot is working correctly.
                                        -------
To use:
 * Hold your hand in front of the obstacle sensors.
 * The robot's head should point left when the left obstacle sensor is triggered
 * The robot's head should point right when the right obstacle sensor is triggered.

 * Hold your hand in front of the line follower sensors.
 * The line follower sensors will move the robot's head up and down.
 
If both obstacle sensors are triggered together, the robot will move forwards.
If both line follower sensors are triggered together, the robot will move backwards.

The ultrasonic sensor is not currently included in this test suite.
*/

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
