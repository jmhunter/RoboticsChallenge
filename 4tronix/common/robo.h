// A set of defines which can be tweaked to reconfigure or tune the 4tronix robot

// Define which Arduino pin is connected to which sensor or servo


// Directional Servo Motor Definitions
#define L1 6   // Motor L1 - pin for the PWM control pin of B motor set to D6
#define L2 12  // Motor L2 - pin for the direction control pin of B motor set to D12
#define L3 5   // Motor L3 - pin for the PWM control pin of A motor set to D5
#define L4 8   // Motor L4 - pin for the direction control pin of A motor set to D8


// Infra Red Obstacle Sensor Definitions
#define OBSTACLE_LEFT  3  // pin for the Line Obstacle Sensor Left set to D3
#define OBSTACLE_RIGHT 2  // pin for the Line Obstacle Sensor Right set to D2


// Line Follow Sensor Definitions
#define LINE_LEFT  A4  // pin for the Line Follow Sensor Left set to A4
#define LINE_RIGHT A5  // pin for the Line Follow Sensor Right set to A5

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
#define ULTRA_SERVO_PAN     11  // pin for the ultrasonic pan motor set to D11
#define ULTRA_SERVO_TILT    10  // pin for the ultrasonic tilt motor set to D10
#define ULTRA_SONAR_ECHO    A0  // pin for the ultrasonic sensor echo set to A0
#define ULTRA_SONAR_TRIGGER A1  // pin for the ultrasonic sensor trigger set to A1

// Ultrasonic Sensor extremities of movement for both the pan and tilt servos
#define ULTRA_SERVO_CENTRE 90

// Ultrasonic Sensor extremities of movement for the pan servo
#define ULTRA_SERVO_PAN_LEFT 135
#define ULTRA_SERVO_PAN_RIGHT 45

// Ultrasonic Sensor extremities of movement for the tilt servo
#define ULTRA_SERVO_TILT_UP 45
#define ULTRA_SERVO_TILT_DOWN 135

// Ultrasonic Sensor servo positional wait time
#define ULTRA_SERVO_WAIT 200

// Utrasonic Sensor sonar ping maximum range
#define ULTRA_SONAR_MAX_RANGE 1000

// Ultrasonic Sensor sonar ping wait time between samples
#define ULTRA_SONAR_WAIT 30

// Ultrasonic Sensor number of times to check the sensor for an average reading
#define ULTRA_SONAR_SAMPLE_SIZE 5
