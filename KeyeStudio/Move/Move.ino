#include "robo.h"
// This is the code sketch to move the KeyeStudio robot around.
//                                     ----------
// Change the code below to make the robot to move how you want it to.
// Start by getting the robot to drive round in a square, a triangle and then a circle.

void loop()
{

  forward(1000,150,150);

  reverse(1000,150,150);

  leftSpin(1000,150);

  rightSpin(1000,150);

}
