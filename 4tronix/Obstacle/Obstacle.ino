#include "robo.h"

// This is the code sketch to make the 4tronix robot follow a BLACK line.
//                                     -------
// Use the red and yellow sensors on the front of the robot to decide what to do in each of the FOUR possible scenarios.
// Change the code below to make the robot move how you want it to.

void loop()
{

  if ( leftObstacleSensor() == false && rightObstacleSensor() == false )
  {
    //What command should you type in here?
     return;
  }




}
