#include "robo.h"

// --- Global Variables ---
RobotType detectedRobot;
PinConfig pins;

// Libraries instances
SR04 sr04_ks = SR04(0, 0); // Placeholder, will be re-initialized if needed
NewPing sonar_4t = NewPing(0, 0, 0); // Placeholder
Servo ultraPanServo, ultraTiltServo;

// Matrix Display Patterns
unsigned char front[] = {0x00,0x40,0x60,0x30,0x18,0x4C,0x66,0x33,0x33,0x66,0x4C,0x18,0x30,0x60,0x40,0x00};
unsigned char right[] = {0x08,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63,0x49,0x1C,0x36,0x63};
unsigned char left[] = {0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x49,0x63,0x36,0x1C,0x08};
unsigned char back[] = {0x00,0x01,0x03,0x06,0x0C,0x19,0x33,0x66,0x66,0x33,0x19,0x0C,0x06,0x03,0x01,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear_pattern[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char eagle[] = {0x00,0x00,0x04,0x0E,0x1E,0x3E,0x3C,0x7D,0x7F,0x3C,0x3E,0x1E,0x0E,0x04,0x00,0x00};

// --- Initialization and Detection ---

void detectRobot() {
  // Try to detect Barclays via I2C pull-ups on A4(18) and A5(19)
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(18, LOW);
  digitalWrite(19, LOW);
  delay(50);
  
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  delay(50);
  
  if (digitalRead(18) == HIGH && digitalRead(19) == HIGH) {
    detectedRobot = KEYESTUDIO;
    Serial.println("Robot Detected: KEYESTUDIO");
    pins = {
      5, 2, 6, 4, // Motor L1, L2, L3, L4
      11, 7, 8,   // LINE_LEFT, CENTRE, RIGHT
      255, 255,   // OBSTACLE_LEFT, RIGHT (none)
      19, 18,     // DISPLAY_CLOCK, ECHO
      17, 255,    // ULTRA_SERVO_PAN, TILT (A3=17)
      13, 12,     // SONAR_ECHO, TRIGGER
      true        // LINE_SENSOR_REVERSED
    };
    sr04_ks = SR04(pins.ULTRA_SONAR_ECHO, pins.ULTRA_SONAR_TRIGGER);
  } else {
    detectedRobot = _4TRONIX;
    Serial.println("Robot Detected: 4TRONIX");
    pins = {
      5, 2, 6, 4, // Motor L1, L2, L3, L4 (aligned)
      11, 255, 8, // LINE_LEFT, CENTRE, RIGHT (aligned, no centre)
      15, 14,     // OBSTACLE_LEFT, RIGHT (A1, A0)
      255, 255,   // DISPLAY (none)
      17, 16,     // ULTRA_SERVO_PAN, TILT (A3, A2)
      13, 12,     // SONAR_ECHO, TRIGGER (aligned)
      true        // LINE_SENSOR_REVERSED
    };
    sonar_4t = NewPing(pins.ULTRA_SONAR_TRIGGER, pins.ULTRA_SONAR_ECHO, ULTRA_SONAR_MAX_RANGE);
  }
}

void initializeServos() {
  if (detectedRobot == _4TRONIX) {
    ultraPanServo.attach(pins.ULTRA_SERVO_PAN);
    ultraTiltServo.attach(pins.ULTRA_SERVO_TILT);
    pointCentre();
    tiltCentre();
  } else {
    // Keyestudio uses manual PWM, but we can also use Servo library if it works.
    // The prompt says "Use one single servo library that works for both robots if possible".
    // Servo library works on most pins.
    ultraPanServo.attach(pins.ULTRA_SERVO_PAN);
    pointCentre();
  }
}

void initializeDisplay() {
  if (detectedRobot == KEYESTUDIO) {
    pinMode(pins.DISPLAY_CLOCK, OUTPUT);
    pinMode(pins.DISPLAY_ECHO, OUTPUT);
    displayClear();
  }
}

// --- Motor Control ---

// Internal helper for raw motor movement
void _rawMove(int l1, int l2, int r1, int r2) {
  analogWrite(pins.L1, l1);
  digitalWrite(pins.L2, l2);
  analogWrite(pins.L3, r1);
  digitalWrite(pins.L4, r2);
}

void robotMove(int left_speed, int right_speed) {
  int l_dir, r_dir;
  int l_pwm, r_pwm;

  // Normalize speed for Keyestudio
  if (detectedRobot == KEYESTUDIO) {
    left_speed = (int)(left_speed * KEYESTUDIO_SPEED_SCALE);
    right_speed = (int)(right_speed * KEYESTUDIO_SPEED_SCALE);
  }

  // Determine directions and PWM based on platform polarity
  if (detectedRobot == KEYESTUDIO) {
    // KS Forward: dir=HIGH, pwm=255-speed; Reverse: dir=LOW, pwm=speed
    l_dir = (left_speed >= 0) ? HIGH : LOW;
    l_pwm = (l_dir == HIGH) ? (255 - abs(left_speed)) : abs(left_speed);
    
    r_dir = (right_speed >= 0) ? HIGH : LOW;
    r_pwm = (r_dir == HIGH) ? (255 - abs(right_speed)) : abs(right_speed);
  } else {
    // 4T Forward: dir=LOW, pwm=speed; Reverse: dir=HIGH, pwm=255-speed
    l_dir = (left_speed >= 0) ? LOW : HIGH;
    l_pwm = (l_dir == HIGH) ? (255 - abs(left_speed)) : abs(left_speed);
    
    r_dir = (right_speed >= 0) ? LOW : HIGH;
    r_pwm = (r_dir == HIGH) ? (255 - abs(right_speed)) : abs(right_speed);
  }

  _rawMove(l_pwm, l_dir, r_pwm, r_dir);
}

void halt(int wait) {
  if (detectedRobot == KEYESTUDIO) matrix_display(eagle);
  Serial.println("Stopping");
  _rawMove(0, LOW, 0, LOW);
  if (wait > 0) delay(wait);
}

void forward(int wait, int vSpeedLeft, int vSpeedRight) {
  if (detectedRobot == KEYESTUDIO) matrix_display(front);
  Serial.println("Moving Forwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(vSpeedLeft, vSpeedRight);
  if (wait > 0) {
    delay(wait);
    halt(0);
  }
}

void reverse(int wait, int vSpeedLeft, int vSpeedRight) {
  if (detectedRobot == KEYESTUDIO) matrix_display(back);
  Serial.println("Moving Backwards: Speed Left and Right: " + String(vSpeedLeft) + " " + String(vSpeedRight));
  robotMove(-vSpeedLeft, -vSpeedRight);
  if (wait > 0) {
    delay(wait);
    halt(0);
  }
}

void leftSpin(int wait, int vSpeed) {
  if (detectedRobot == KEYESTUDIO) matrix_display(left);
  Serial.println("Spinning left");
  robotMove(-vSpeed, vSpeed);
  if (wait > 0) {
    delay(wait);
    halt(0);
  }
}

void rightSpin(int wait, int vSpeed) {
  if (detectedRobot == KEYESTUDIO) matrix_display(right);
  Serial.println("Spinning right");
  robotMove(vSpeed, -vSpeed);
  if (wait > 0) {
    delay(wait);
    halt(0);
  }
}

// --- Servo Control ---

void pointCentre() {
  ultraPanServo.write(ULTRA_SERVO_CENTRE);
  delay(ULTRA_SERVO_WAIT);
}

void pointLeft() {
  ultraPanServo.write(ULTRA_SERVO_PAN_LEFT);
  delay(ULTRA_SERVO_WAIT);
}

void pointRight() {
  ultraPanServo.write(ULTRA_SERVO_PAN_RIGHT);
  delay(ULTRA_SERVO_WAIT);
}

void pointValue(int pos) {
  ultraPanServo.write(pos);
  delay(ULTRA_SERVO_WAIT);
}

void tiltCentre() {
  if (detectedRobot == _4TRONIX) {
    ultraTiltServo.write(ULTRA_SERVO_CENTRE);
    delay(ULTRA_SERVO_WAIT);
  }
}

void tiltUp() {
  if (detectedRobot == _4TRONIX) {
    ultraTiltServo.write(ULTRA_SERVO_TILT_UP);
    delay(ULTRA_SERVO_WAIT);
  }
}

void tiltDown() {
  if (detectedRobot == _4TRONIX) {
    ultraTiltServo.write(ULTRA_SERVO_TILT_DOWN);
    delay(ULTRA_SERVO_WAIT);
  }
}

void tiltValue(int pos) {
  if (detectedRobot == _4TRONIX) {
    ultraTiltServo.write(pos);
    delay(ULTRA_SERVO_WAIT);
  }
}

// --- Line Sensors ---

int leftLineSensor() {
  int val = digitalRead(pins.LINE_LEFT);
  Serial.println("Left Line Sensor: " + String(val));
  return val;
}

int centreLineSensor() {
  if (pins.LINE_CENTRE == 255) return 255;
  int val = digitalRead(pins.LINE_CENTRE);
  Serial.println("Centre Line Sensor: " + String(val));
  return val;
}

int rightLineSensor() {
  int val = digitalRead(pins.LINE_RIGHT);
  Serial.println("Right Line Sensor: " + String(val));
  return val;
}

// --- Obstacle Sensors ---

boolean leftObstacleSensor() {
  if (detectedRobot == _4TRONIX) {
    if (digitalRead(pins.OBSTACLE_LEFT) == 0) {
      Serial.println("Left Obstacle Detected");
      return true;
    }
    return false;
  } else {
    pointLeft();
    return (Ultrasonic() < ULTRA_SONAR_THRESHOLD);
  }
}

boolean rightObstacleSensor() {
  if (detectedRobot == _4TRONIX) {
    if (digitalRead(pins.OBSTACLE_RIGHT) == 0) {
      Serial.println("Right Obstacle Detected");
      return true;
    }
    return false;
  } else {
    pointRight();
    return (Ultrasonic() < ULTRA_SONAR_THRESHOLD);
  }
}

int Ultrasonic() {
  int cm = 0;
  if (detectedRobot == KEYESTUDIO) {
    cm = sr04_ks.Distance();
  } else {
    unsigned int pingTime = sonar_4t.ping();
    cm = pingTime / US_ROUNDTRIP_CM;
    if (cm == 0 || cm > ULTRA_SONAR_MAX_RANGE) cm = ULTRA_SONAR_MAX_RANGE;
    delay(ULTRA_SONAR_WAIT);
  }
  Serial.print("Sonar Ping: ");
  Serial.print(cm);
  Serial.println("cm");
  return cm;
}

// --- Display ---

unsigned char last_matrix_value[16] = {0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE};

void IIC_start() {
  digitalWrite(pins.DISPLAY_CLOCK, HIGH);
  delayMicroseconds(3);
  digitalWrite(pins.DISPLAY_ECHO, HIGH);
  delayMicroseconds(3);
  digitalWrite(pins.DISPLAY_ECHO, LOW);
  delayMicroseconds(3);
}

void IIC_send(unsigned char send_data) {
  for (char i = 0; i < 8; i++) {
    digitalWrite(pins.DISPLAY_CLOCK, LOW);
    delayMicroseconds(3);
    if (send_data & 0x01) digitalWrite(pins.DISPLAY_ECHO, HIGH);
    else digitalWrite(pins.DISPLAY_ECHO, LOW);
    delayMicroseconds(3);
    digitalWrite(pins.DISPLAY_CLOCK, HIGH);
    delayMicroseconds(3);
    send_data = send_data >> 1;
  }
}

void IIC_end() {
  digitalWrite(pins.DISPLAY_CLOCK, LOW);
  delayMicroseconds(3);
  digitalWrite(pins.DISPLAY_ECHO, LOW);
  delayMicroseconds(3);
  digitalWrite(pins.DISPLAY_CLOCK, HIGH);
  delayMicroseconds(3);
  digitalWrite(pins.DISPLAY_ECHO, HIGH);
  delayMicroseconds(3);
}

void matrix_display(unsigned char matrix_value[]) {
  if (detectedRobot != KEYESTUDIO) return;

  // Check if pattern has changed
  bool changed = false;
  for (int i = 0; i < 16; i++) {
    if (matrix_value[i] != last_matrix_value[i]) {
      changed = true;
      break;
    }
  }
  if (!changed) return;

  // Update cache
  for (int i = 0; i < 16; i++) last_matrix_value[i] = matrix_value[i];

  // Original IIC Sequence
  IIC_start();
  IIC_send(0xc0); // Address command
  for (int i = 0; i < 16; i++) {
    IIC_send(matrix_value[i]);
  }
  IIC_end();

  IIC_start();
  IIC_send(0x8A); // Display control
  IIC_end();
}

void displayClear() {
  matrix_display(clear_pattern);
}

void displayOff() {
  displayClear();
}

void displayNumber(int num) {
  // Logic to display number could be added here, using existing patterns or new ones.
  // For now, just clear as a placeholder or use a simple pattern.
  displayClear();
}

// --- Celebration ---

void jiggleBot(int repeats) {
  for (int i = 0; i < repeats; i++) {
    forward(200, 255, 255);
    reverse(200, 255, 255);
  }
}

void wiggleBot(int repeats) {
  for (int i = 0; i < repeats; i++) {
    leftSpin(200, 255);
    rightSpin(400, 255);
    leftSpin(200, 255);
  }
}

// --- Standard Arduino Entry Points ---

void setup() {
  Serial.begin(115200);
  detectRobot();
  
  // Set Pin Modes
  pinMode(pins.L1, OUTPUT);
  pinMode(pins.L2, OUTPUT);
  pinMode(pins.L3, OUTPUT);
  pinMode(pins.L4, OUTPUT);
  
  pinMode(pins.LINE_LEFT, INPUT);
  if (pins.LINE_CENTRE != 255) pinMode(pins.LINE_CENTRE, INPUT);
  pinMode(pins.LINE_RIGHT, INPUT);
  
  if (pins.OBSTACLE_LEFT != 255) pinMode(pins.OBSTACLE_LEFT, INPUT);
  if (pins.OBSTACLE_RIGHT != 255) pinMode(pins.OBSTACLE_RIGHT, INPUT);
  
  initializeServos();
  initializeDisplay();
  
  Serial.println("Robot Initialized");
}
