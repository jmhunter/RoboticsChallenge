// A set of defines which can be tweaked to reconfigure or tune the KeyeStudio robot

// Define which Arduino pin is connected to what device


// Directional Servo Motor Definitions
#define L1 5  // Motor L1 - pin for the PWM control pin of B motor set to D5
#define L2 4  // Motor L2 - pin for the direction control pin of B motor set to D4
#define L3 9  // Motor L3 - pin for the PWM control pin of A motor set to D9
#define L4 2  // Motor L4 - pin for the direction control pin of A motor set to D2


// Line Follow Sensor Definitions
const int LINE_LEFT = 6;    // pin for the Line Follow Sensor Left set to D6
const int LINE_CENTRE = 7;  // pin for the Line Follow Sensor Middle set to D7
const int LINE_RIGHT = 8;   // pin for the Line Follow Sensor Right set to D8
int line_left_val,line_middle_val,line_right_val;  // returning values from line sensors

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

// Inclusion of libraries for control of robot servos and sensors
#include "SR04.h"    // Ultrasonic Sensor library for KeyeStudio robots

// SR04 library for setup of Ultrasonic Sensor
SR04 sr04 = SR04(ULTRA_SONAR_ECHO,ULTRA_SONAR_TRIGGER);

// Ultrasonic Sensor extremities of movement for the pan servo
#define ULTRA_SERVO_CENTRE     90  // the centre position in degrees for the pan servo
#define ULTRA_SERVO_PAN_LEFT  135  // the left half way position in degrees for the pan servo
#define ULTRA_SERVO_PAN_RIGHT  45  // the right half way position in degrees for the pan servo

// Ultrasonic Sensor pan servo positional wait time
#define ULTRA_SERVO_PAN_WAIT 200

// Utrasonic Sensor sonar ping
long ULTRA_SONAR_DISTANCE,distance_left,distance_right;  // variables for measuring distance using the Ultrasonic sensor

// Ultrasonic Sensor sonar ping wait time between samples
#define ULTRA_SONAR_WAIT 30

// Ultrasonic Sensor number of times to check the sensor for an average reading
#define ULTRA_SONAR_SAMPLE_SIZE 5

// What distance counts as 'there is an obstacle'
#define ULTRA_SONAR_THRESHOLD 50


// Matrix Display Definitions
#define DISPLAY_CLOCK A5  // pin for the matrix display clock set to A5
#define DISPLAY_ECHO  A4  // pin for the matrix display data set to A4
unsigned char front[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char back[] = {0x00,0x00,0x00,0x00,0x00,0x09,0x12,0x24,0x12,0x09,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
