//A set of defines which can be tweaked to reconfigure or tune the KeyeStudio robot

// Define which Arduino pin is connected to what device


// Directional Servo Motor Definitions
#define L1 5  // Motor L1 - pin for the PWM control pin of B motor set to D5
#define L2 4  // Motor L2 - pin for the direction control pin of B motor set to D4
#define L3 9  // Motor L3 - pin for the PWM control pin of A motor set to D9
#define L4 2  // Motor L4 - pin for the direction control pin of A motor set to D2


// Line Follow Sensor Definitions
const int LINE_LEFT = 6;    // pin for the Line Follow Sensor Left set to D6
const int LINE_MIDDLE = 7;  // pin for the Line Follow Sensor Middle set to D7
const int LINE_RIGHT = 8;   // pin for the Line Follow Sensor Right set to D8
int line_left_val,line_middle_val,line_right_val;  //define these variables

// If the line follower code needs reversing then set this (new sensors are reversed!)
#define REVERSE_LF true
#ifdef REVERSE_LF
  #define BLACK 1
  #define WHITE 0
#else
  #define BLACK 0
  #define WHITE 1
#endif

// Line Follower Sensor threshold to use between BLACK or WHITE
#define LINE_THRESHOLD 500

// Line Follower Sensor number of times to check the sensor for an average reading
#define LINE_SAMPLE_SIZE 1


// Ultrasonic Sensor Definitions
const int ULTRA_SERVO_PAN = 10; // pin for the ultrasonic pan motor set to D10
#define ULTRA_SONAR_ECHO    13  // pin for the ultrasonic sensor echo set to D13
#define ULTRA_SONAR_TRIGGER 12  // pin for the ultrasonic sensor trigger set to D12

// Ultrasonic Sensor extremities of movement for the pan servo
#define ULTRA_SERVO_PAN_LEFT  180  // the left-most position in degrees for the pan servo
#define ULTRA_SERVO_PAN_CENTRE 90  // the centre position in degrees for the pan servo
#define ULTRA_SERVO_PAN_RIGHT   0  // the right-most position in degrees for the pan servo

// Ultrasonic Sensor pan servo positional wait time
#define ULTRA_SERVO_PAN_WAIT 200

// Utrasonic Sensor sonar ping maximum range
// #define ULTRA_SONAR_MAX_RANGE 1000 // might not be supported by SR04 library

// Ultrasonic Sensor sonar ping wait time between samples
#define ULTRA_SONAR_WAIT 30

// Ultrasonic Sensor number of times to check the sensor for an average reading
#define ULTRA_SONAR_SAMPLE_SIZE 5


// Matrix Display Definitions
#define MATRIX_CLOCK   A5  // pin for the matrix display clock set to A5
#define MATRIX_DISPLAY A4  // pin for the matrix display data set to A4
