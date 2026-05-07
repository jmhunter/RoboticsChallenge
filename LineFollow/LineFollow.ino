#include"robo.h"

// This is the code sketch to make the Barclays robot follow a BLACK line.
//                                     
// Use the sensors underneath the robot to decide what to do in each of the FOUR possible scenarios.
// Change the code below to make the robot move how you want it to.

void loop()
{

  if (leftLineSensor() == BLACK && rightLineSensor() == BLACK )
  {
    //What command should you type in here?
    forward(10,255,255);
    return;
  }

if (leftLineSensor() == WHITE && rightLineSensor() == BLACK )
  {
    rightSpin(10,255);
    return;
  }

if (leftLineSensor() == BLACK && rightLineSensor() == WHITE )
  {
    leftSpin(10,255);
    return;
  }




}
