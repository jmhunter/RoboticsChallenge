#include "robo.h"

// This is the code sketch to make the KeyeStudio robot navigate an obstacle course.
//                                     ----------
// Use the sensors on the robot's head to decide what to do in each of the FOUR possible scenarios.
// Change the code below to make the robot move how you want it to.

void loop()
{
  ULTRA_SONAR_DISTANCE=sr04.Distance(); //obtain the value detected by ultrasonic sensor

  if((ULTRA_SONAR_DISTANCE < 40)&&(ULTRA_SONAR_DISTANCE > 0))//if the distance is greater than 0 and less than 20  
    {
      halt(0);                                                // stop the robot
      matrix_display(STOP01);                                 // show the stop pattern on the led matrix display
      ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_LEFT);  // rotate pan servo to the left
      distance_left=sr04.Distance();                          // measure the left distance to nearest obstacle
      delay(ULTRA_SONAR_WAIT);
      ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_RIGHT); // rotate pan servo to the right
      distance_right=sr04.Distance();                         // measure the right distance to nearest obstacle
      delay(ULTRA_SONAR_WAIT);

    if(distance_left > distance_right)   // if distance to the left is greater than distance to the right
      {
        leftSpin(200,255);                                    // turn robot to the left
        matrix_display(left);                                 // show the left turn pattern on the led matrix display
        ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);  // rotate pan servo back to the centre
        matrix_display(front);                                // show the forward pattern on the led matrix display
      }
      else                             // if the right distance is greater than the left
      {
        rightSpin(200,255);                                   // turn robot to the right
        matrix_display(right);                                // show the left turn pattern on the led matrix display
        ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);  // rotate pan servo back to the centre
        matrix_display(front);                                // show the forward pattern on the led matrix display
      }
    }
  else                               //otherwise
  {
    forward(50,255,255);   // move the robot forward
    matrix_display(front);  // show the forward pattern on the led matrix display
  }
}
