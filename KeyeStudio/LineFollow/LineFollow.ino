#include"robo.h"

// This is the code sketch to make the KeyeStudio robot follow a BLACK line.
//                                     ----------
// Use the sensors underneath the robot to decide what to do in each of the FOUR possible scenarios.
// Change the code below to make the robot move how you want it to.

void loop()
{

  if (leftLineSensor() == BLACK && rightLineSensor() == BLACK )
  {
    forward(1000,255,255);
    return;
  }




}
