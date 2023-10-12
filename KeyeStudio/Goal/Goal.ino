#include "robo.h"

// This is the code sketch to make the KeyeStudio robot score a goal.
//                                     ----------
// Use the sensors on the robot's head to work out where the ball is and how far away, then move the robot behind the ball.
// When the robot is behind the ball, shoot for the goal!
//
// Change the code below to make the robot move how you want it to.

void loop()
{
  // Reset all the distance values to zero
  int leftDistance=0;
  int centreDistance=0;
  int rightDistance=0;


  // point robot's head to the left
  pointLeft();
  
  // read the distance on the robot's left using the Ultrasonic sensor
  leftDistance = Ultrasonic();
  
  // Where do you want to turn the robot's head next?




// Once you have taken all the readings, how do you want the robot to react?

  if ( leftDistance <= centreDistance && leftDistance <= rightDistance )
  {
    //What command should you type in here?
  }

  // What other distance options do you need to check?






  // Celebrate the goal by running the goal celebration routine
goalCelebration();

  // Have a rest
halt(5000);

}



// Change the goal celebration here
void goalCelebration()
{
  jiggleBot(3);
  wiggleBot(4);
}
