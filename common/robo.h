#ifndef ROBO_H
#define ROBO_H

#include <Arduino.h>
#include <Servo.h>
#include "NewPing.h"
#include "SR04.h"

// --- Robot Detection ---
enum RobotType { KEYESTUDIO, _4TRONIX };
extern RobotType detectedRobot;

// --- Pin Configuration ---
struct PinConfig {
  byte L1, L2, L3, L4;  // Motor pins
  byte LINE_LEFT, LINE_CENTRE, LINE_RIGHT;  // Line sensors
  byte OBSTACLE_LEFT, OBSTACLE_RIGHT;  // Obstacle sensors (4tronix only)
  byte DISPLAY_CLOCK, DISPLAY_ECHO;  // Display (KeyeStudio only)
  byte ULTRA_SERVO_PAN, ULTRA_SERVO_TILT;  // Servo pins
  byte ULTRA_SONAR_ECHO, ULTRA_SONAR_TRIGGER;  // Sonar pins
  boolean LINE_SENSOR_REVERSED;  // Line sensor inversion
};

extern PinConfig pins;

// --- Speed Scaling ---
#define KEYESTUDIO_SPEED_SCALE 0.51  // 130/255 approx 0.51

// Constants
#define BLACK 1
#define WHITE 0

// Ultrasonic Constants
#define ULTRA_SERVO_CENTRE     90
#define ULTRA_SERVO_PAN_LEFT  135
#define ULTRA_SERVO_PAN_RIGHT  45
#define ULTRA_SERVO_TILT_UP    45
#define ULTRA_SERVO_TILT_DOWN 135
#define ULTRA_SERVO_WAIT      200
#define ULTRA_SONAR_WAIT       30
#define ULTRA_SONAR_SAMPLE_SIZE 5
#define ULTRA_SONAR_THRESHOLD  50
#define ULTRA_SONAR_MAX_RANGE 1000

// Matrix Display Patterns
extern unsigned char front[];
extern unsigned char right[];
extern unsigned char left[];
extern unsigned char back[];
extern unsigned char STOP01[];
extern unsigned char clear_pattern[];
extern unsigned char eagle[];

// --- Function Prototypes ---

// Initialization and Detection
void detectRobot();
void initializeServos();
void initializeDisplay();

// Motor Control
void robotMove(int l_speed, int l_dir, int r_speed, int r_dir);
void halt(int wait = 0);
void forward(int wait, int vSpeedLeft, int vSpeedRight);
void reverse(int wait, int vSpeedLeft, int vSpeedRight);
void leftSpin(int wait, int vSpeed);
void rightSpin(int wait, int vSpeed);

// Servo Control
void pointCentre();
void pointLeft();
void pointRight();
void pointValue(int pos);
void tiltCentre();
void tiltUp();
void tiltDown();
void tiltValue(int pos);

// Line Sensors
int leftLineSensor();
int centreLineSensor();
int rightLineSensor();

// Obstacle Sensors
boolean leftObstacleSensor();
boolean rightObstacleSensor();
int Ultrasonic();

// Display
void displayNumber(int num);
void displayClear();
void displayOff();
void matrix_display(unsigned char matrix_value[]);

// Celebration
void jiggleBot(int repeats);
void wiggleBot(int repeats);

#endif
