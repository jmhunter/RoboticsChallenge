#include "robo.h"

// This is the code sketch to make the KeyeStudio robot navigate an obstacle course.
//                                     ----------
// Use the sensors on the robot's head to decide what to do in each of the FOUR possible scenarios.
// Change the code below to make the robot move how you want it to.

void loop()
{

  if ( leftObstacleSensor() == false && rightObstacleSensor() == false )
  {
    //What command should you type in here?
    return;
  }




}
