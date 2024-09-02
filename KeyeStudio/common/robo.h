// A set of defines which can be tweaked to reconfigure or tune the KeyeStudio robot

// Define which Arduino pin is connected to what device

// Which robot variant is this?
#define ROBOTMODEL KEYESTUDIO_0559
//#define ROBOTMODEL KEYESTUDIO_0470
//#define ROBOTMODEL 4TRONIX


#if ROBOTMODEL == KEYESTUDIO_0559
// Directional Servo Motor Definitions
#define L1 5  // Motor L1 - pin for the PWM control pin of B motor
#define L2 2  // Motor L2 - pin for the direction control pin of B motor
#define L3 6  // Motor L3 - pin for the PWM control pin of A motor
#define L4 4  // Motor L4 - pin for the direction control pin of A motor


// Line Follow Sensor Definitions
const int LINE_LEFT = 11;    // pin for the Line Follow Sensor Left
const int LINE_CENTRE = 7;  // pin for the Line Follow Sensor Middle
const int LINE_RIGHT = 8;   // pin for the Line Follow Sensor Right

// If the line follower code needs reversing then set this (new sensors are reversed!)
#define REVERSE_LF true

// Ultrasonic Sensor Definitions
const int ULTRA_SERVO_PAN = A3; // pin for the ultrasonic pan motor
#define ULTRA_SONAR_ECHO    13  // pin for the ultrasonic sensor echo
#define ULTRA_SONAR_TRIGGER 12  // pin for the ultrasonic sensor trigger


#elif ROBOTMODEL == KEYESTUDIO_0470
// Directional Servo Motor Definitions
#define L1 5  // Motor L1 - pin for the PWM control pin of B motor set to D5
#define L2 4  // Motor L2 - pin for the direction control pin of B motor set to D4
#define L3 9  // Motor L3 - pin for the PWM control pin of A motor set to D9
#define L4 2  // Motor L4 - pin for the direction control pin of A motor set to D2


// Line Follow Sensor Definitions
const int LINE_LEFT = 6;    // pin for the Line Follow Sensor Left set to D6
const int LINE_CENTRE = 7;  // pin for the Line Follow Sensor Middle set to D7
const int LINE_RIGHT = 8;   // pin for the Line Follow Sensor Right set to D8

// If the line follower code needs reversing then set this (new sensors are reversed!)
#define REVERSE_LF true

// Ultrasonic Sensor Definitions
const int ULTRA_SERVO_PAN = 10; // pin for the ultrasonic pan motor
#define ULTRA_SONAR_ECHO    13  // pin for the ultrasonic sensor echo
#define ULTRA_SONAR_TRIGGER 12  // pin for the ultrasonic sensor trigger


#elif ROBOTMODEL == 4TRONIX

// TODO XXXX
// Merge in 4tronix code
// Also move matrix LED definitions further up in this file, as 4tronix doesn't have

// Ultrasonic Sensor Definitions
const int ULTRA_SERVO_PAN = 11; // pin for the ultrasonic pan motor
#define ULTRA_SONAR_ECHO    A0  // pin for the ultrasonic sensor echo
#define ULTRA_SONAR_TRIGGER A1  // pin for the ultrasonic sensor trigger


#endif

#ifdef REVERSE_LF
  #define BLACK 1
  #define WHITE 0
#else
  #define BLACK 0
  #define WHITE 1
#endif

int line_left_val,line_middle_val,line_right_val;  // returning values from line sensors

// Line Follower Sensor threshold to use between BLACK or WHITE
#define LINE_THRESHOLD 500

// Line Follower Sensor number of times to check the sensor for an average reading
#define LINE_SAMPLE_SIZE 1


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

// Ultrasonic Sensor sonar ping wait time between samples
#define ULTRA_SONAR_WAIT 30

// Ultrasonic Sensor number of times to check the sensor for an average reading
#define ULTRA_SONAR_SAMPLE_SIZE 5

// Ultrasonic Sensor threshold for what distance counts as 'there is an obstacle present'
#define ULTRA_SONAR_THRESHOLD 50


// Matrix Display Definitions
#define DISPLAY_CLOCK A5  // pin for the matrix display clock set to A5
#define DISPLAY_ECHO  A4  // pin for the matrix display data set to A4
unsigned char front[] = {0x00,0x40,0x60,0x30,0x18,0x4C,0x66,0x33,0x33,0x66,0x4C,0x18,0x30,0x60,0x40,0x00};
unsigned char right[] = {0x08,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63};
unsigned char left[] = {0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x08};
unsigned char back[] = {0x00,0x01,0x03,0x06,0x0C,0x19,0x33,0x66,0x66,0x33,0x19,0x0C,0x06,0x03,0x01,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char eagle[] = {0x00,0x00,0x04,0x0E,0x1E,0x3E,0x3C,0x7D,0x7F,0x3C,0x3E,0x1E,0x0E,0x04,0x00,0x00};

// Used for debugging line follower sensors
unsigned char lf_bb[] = {0x00,0x3F,0x25,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x25,0x1A,0x00 };
unsigned char lf_wb[] = {0x00,0x3F,0x25,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x30,0x0C,0x30,0x0F };
unsigned char lf_bw[] = {0x00,0x0F,0x30,0x0C,0x30,0x0F,0x00,0x00,0x00,0x00,0x00,0x3F,0x25,0x1A,0x00,0x00 };
unsigned char lf_ww[] = {0x00,0x0F,0x30,0x0C,0x30,0x0F,0x00,0x00,0x00,0x00,0x00,0x0F,0x30,0x0C,0x30,0x0F };
