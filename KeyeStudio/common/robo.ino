/*

  Robotics Challenge - Version History

  v 1.0 - 15/03/14  - Backend code for the robotics - same code master - will be overwritten
  v 1.1 - 21/08/14  - Updated to include new challenges
  v 1.2 - 23/03/15  - Cleaned up and simplified
  v 1.3 - 10/05/16  - Updated to fix minor bugs.  Also has a TestAll function you use to
                      test all components are working
  v 1.4 - 20/09/23  - Complete re-write to support KeyeStudio robots now the orginal 4tronix
                      robots are no longer available

*/

// robo sets up all the functions for all robot movements and sensors
// Look in robo.h for all the #defines


// The setup routine runs once when you press reset
// This must be here to set your pins for the motor and do some initialisation
void setup()
{
// initialize the digital pins to be used as an output for the Motors
  pinMode(L1, OUTPUT);  // define PWM control pin of B motor as OUTPUT
  pinMode(L2, OUTPUT);  // define direction control pin of B motor as OUTPUT
  pinMode(L3, OUTPUT);  // define PWM control pin of A motor as OUTPUT
  pinMode(L4, OUTPUT);  // define direction control pin of A motor as OUTPUT

// initialize the digital pins to be used as an input for the Line Following Sensors
  pinMode(LINE_LEFT,   INPUT);  // define the pin of left line tracking sensor as INPUT
  pinMode(LINE_CENTRE, INPUT);  // define the pin of middle line tracking sensor as INPUT
  pinMode(LINE_RIGHT, INPUT);   // define the pin of right line tracking sensor as INPUT

// initialize the digital pins to be used as an output for the Matrix Display Screen
  pinMode(DISPLAY_CLOCK, OUTPUT);  // define the pin of matrix clock as OUTPUT
  pinMode(DISPLAY_ECHO,  OUTPUT);  // define the pin of matrix display to OUTPUT

// Start the Serial device and make sure the speed is set to 115,200 baud
  Serial.begin(115200);
  Serial.println("Starting Barclays Robot");

// Centre the Ultrasonic sensor servo as part of the robot set up routine
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);  // Set the angle of Ultrasonic servo to 90 degrees
  delay(ULTRA_SERVO_PAN_WAIT);                          // Match 4Tronix robot behaviour by adding startup delay
}


// robotMove routine provides the inputs to the motors for basic robot movement
void robotMove(int l1, int l2, int r1, int r2)
{
#if ROBOTMODEL == KEYESTUDIO_0559
  // For Keyestudio 0559 robots, the PWM must be adjusted for the reverse direction (!!!)
  if(l2==HIGH) l1=255-l1;
  if(r2==HIGH) r1=255-r1;
 #endif

  analogWrite(L1, l1);
  digitalWrite(L2, l2);
  analogWrite(L3, r1);
  digitalWrite(L4, r2);
}


// forward basic movement routine
void forward(int wait,int vSpeedLeft,int vSpeedRight)

// ensures no value over 255 can be used for forward movement
{
  if ( vSpeedLeft > 255 )
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 )
    vSpeedRight = 255;

  matrix_display(front);                           // show the forward pattern on the LED matrix display
  Serial.println("Moving Forwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(vSpeedLeft, HIGH, vSpeedRight, HIGH);  // turn the left and right motors forwards
  delay(wait);

//stop forward movement routine after the wait time
  halt(0);
}


// reverse basic movement routine
void reverse(int wait, int vSpeedLeft, int vSpeedRight)

// ensures no value over 255 can be used for reverse movement
{
  if ( vSpeedLeft > 255 )
    vSpeedLeft = 255;

  if ( vSpeedRight > 255 )
    vSpeedRight = 255;

  matrix_display(back);                          // show the reverse pattern on the LED matrix display
  Serial.println("Moving Backwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(vSpeedLeft, LOW, vSpeedRight, LOW);  // turn the left and right motors backwards
  delay(wait);

//stop reverse movement routine after the wait time
  halt(0);
}


// turning left basic movement routine
void leftSpin(int wait,int vSpeed)
{
  matrix_display(left);                  // show the left turn pattern on the LED matrix display
  Serial.println("Spinning left");
  robotMove(vSpeed, LOW, vSpeed, HIGH);  // turn the left motors backwards and the right motors forwards
  delay(wait);
  
//stop turning left movement routine after the wait time
  halt(0);
}


// turning right basic movement routine
void rightSpin(int wait, int vSpeed)
{
  matrix_display(right);                 // show the right turn pattern on the LED matrix display
  Serial.println("Spinning right");
  robotMove(vSpeed, HIGH, vSpeed, LOW);  // turn the right motors backwards and the left motors forwards
  delay(wait);

//stop turning right movement routine after the wait time
  halt(0);
}


// stopping all basic movement routine
void halt(int wait)
{
  matrix_display(eagle);                 // show the right turn pattern on the LED matrix display
  Serial.println("Stopping");
  robotMove(0, LOW, 0, LOW);             // stop both left and right motors
  delay(wait);
}


// IIC routines are used for the LED dot matrix display
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();                  // the function to call the data transmission
  IIC_send(0xc0);               // select address
  for(int i = 0;i < 16;i++)     // Pattern data has 16 bytes
  {
    IIC_send(matrix_value[i]);  // data to convey patterns
  }
  IIC_end();                    // end the transmission of patterns data
  IIC_start();
  IIC_send(0x8A);               // display control, set pulse width to 4/16
  IIC_end();
}

// the condition that data transmission starts  
void IIC_start()
{
  digitalWrite(DISPLAY_CLOCK,HIGH);
  delayMicroseconds(3);
  digitalWrite(DISPLAY_ECHO,HIGH);
  delayMicroseconds(3);
  digitalWrite(DISPLAY_ECHO,LOW);
  delayMicroseconds(3);
}

// transmit data to the LED dot matrix display
void IIC_send(unsigned char send_data)
{
  for(char i = 0;i < 8;i++)           // every character has 8 bits
  {
    digitalWrite(DISPLAY_CLOCK,LOW);  // pull down the DISPLAY_CLOCK to change the signal of SDA
    delayMicroseconds(3);
    if(send_data & 0x01)              // 1 or 0 of byte is used to set high and low level of DISPLAY_ECHO
    {
      digitalWrite(DISPLAY_ECHO,HIGH);
    }
    else
    {
      digitalWrite(DISPLAY_ECHO,LOW);
    }
    delayMicroseconds(3);
    digitalWrite(DISPLAY_CLOCK,HIGH); // pull up DISPLAY_CLOCK to stop data transmission
    delayMicroseconds(3);
    send_data = send_data >> 1;       // detect bit by bit, so move the data right by one bit
  }
}

// the sign that data transmission ends 
void IIC_end()
{
  digitalWrite(DISPLAY_CLOCK,LOW);
  delayMicroseconds(3);
  digitalWrite(DISPLAY_ECHO,LOW);
  delayMicroseconds(3);
  digitalWrite(DISPLAY_CLOCK,HIGH);
  delayMicroseconds(3);
  digitalWrite(DISPLAY_ECHO,HIGH);
  delayMicroseconds(3);
}


// LineSensor routines provide the inputs to the line sensors to detect dark lines on a lighter background

int readLineSensor(int pin)
{
  return digitalRead(pin);
}

// Takes the reading from the left line sensor and returns whether it is BLACK or WHITE
int leftLineSensor()
{
  int val= readLineSensor(LINE_LEFT);
  Serial.println("Left Line Sensor: " + String(val));
  return val;
}

// Takes the reading from the right line sensor and returns whether it is BLACK or WHITE
int rightLineSensor()
{
  int val = readLineSensor(LINE_RIGHT);
  Serial.println("Right Line Sensor: " + String(val));
  return val;
}


// Takes the reading from the centre line sensor and returns whether it is BLACK or WHITE
int centreLineSensor()
{
  int val= readLineSensor(LINE_CENTRE);
  Serial.println("Centre Line Sensor: " + val);
  return val;
}


// UltraSonic routines provide the inputs to the Ultrasonic sensor and pan motor for detection and movement of the Ultrasonic head

// Calculates the running angle of the Ultrasonic Pan Servo
void ultraServoPulse(int ULTRA_SERVO_PAN,int myangle)
{
  for(int i=0; i<30; i++)
  {
    int pulseWidth = (myangle*11)+500;
    digitalWrite(ULTRA_SERVO_PAN,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(ULTRA_SERVO_PAN,LOW);
    delay(20-pulseWidth/1000);
  }
}

// Point the Ultrasonic pan servo to the left
int pointLeft()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_LEFT);
}

// Point the Ultrasonic pan servo to the right
int pointRight()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_PAN_RIGHT);
}

// Point the Ultrasonic pan servo to the centre
int pointCentre()
{
  ultraServoPulse(ULTRA_SERVO_PAN,ULTRA_SERVO_CENTRE);
}

// Point the Ultrasonic pan servo left, take the reading from the Ultrasonic sensor and return whether it is TRUE or FALSE for the presence of an obstacle
int leftObstacleSensor()
{
  matrix_display(STOP01);                         // show the STOP pattern on the led matrix display
  pointLeft();                                    // rotate pan servo to the left
  return (Ultrasonic() < ULTRA_SONAR_THRESHOLD);  // take an Ultrasonic sonar reading and compare it to the threshold value
}

// Point the Ultrasonic pan servo right, take the reading from the Ultrasonic sensor and return whether it is TRUE or FALSE for the presence of an obstacle
int rightObstacleSensor()
{
  matrix_display(STOP01);                         // show the STOP pattern on the led matrix display
  pointRight();                                     // rotate pan servo to the right
  return (Ultrasonic() < ULTRA_SONAR_THRESHOLD);  // take an Ultrasonic sonar reading and compare it to the threshold value
}

// Take a distance reading using the Ultrasonic sensor and return the distance value
int Ultrasonic()
{
  return (sr04.Distance() );  // measure the distance to an obstacle using the Ultrasonic sensor
}


// Robot celebration routines

// Jiggle the robot forward and backward for the number of recursions specified
void jiggleBot (int repeats)
{
  for (int i=0; i<repeats; i++)
  {
    forward(200, 255, 255);
    reverse(200, 255, 255);
  }
}

// Wiggle the robot left and right for the number of recursions specified
void wiggleBot (int repeats)
{
  int i = 0;

  while (i < repeats)
  {
    leftSpin(200, 255);
    rightSpin(400, 255);
    leftSpin(200,255);

    i++;
  }
}
