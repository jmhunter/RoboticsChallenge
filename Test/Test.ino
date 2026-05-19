#include "robo.h"

// Override debug level for the diagnostic suite to show all sensor data
// #undef DEBUG_LEVEL
// #define DEBUG_LEVEL 3

/* 
 * Barclays Robot Unified Interactive Diagnostic Suite
 * --------------------------------------------------
 * This test suite responds to individual sensor triggers:
 * 
 * 1. OBSTACLE SENSORS:
 *    - Left only: Point head Left (+ small spin on KS).
 *    - Right only: Point head Right (+ small spin on KS).
 *    - Both (or proxy on KS): Full movement sequence (Fwd, Back, L, R).
 * 
 * 2. LINE SENSORS:
 *    - Left: Point head Right (to "track" the line).
 *    - Right: Point head Left.
 *    - Centre (KS) or Both L/R (4T): Move Forward slightly.
 * 
 * Gracefully adapts to hardware differences (Tilt, Centre Line, IR).
 */

void loop() {
  // Read Line Sensors
  int L = leftLineSensor();
  int C = centreLineSensor();
  int R = rightLineSensor();
  
  // Read Obstacle Sensors
  bool obsL = false;
  bool obsR = false;
  
  if (detectedRobot == _4TRONIX) {
    obsL = leftObstacleSensor();
    obsR = rightObstacleSensor();
  } else {
    // KeyeStudio proxy: use Ultrasonic to detect "hand" in front
    // We only simulate Obstacle triggers if something is very close (< 15cm)
    int dist = Ultrasonic();
    if (dist < 15) {
      // If we are pointing centre, we assume it's "both"
      obsL = true; obsR = true;
    }
  }

  // --- 1. Both Obstacles -> Full Dance ---
  if (obsL && obsR) {
    Serial.println("DIAG: Both Obstacles -> Movement Sequence");
    if (detectedRobot == KEYESTUDIO) matrix_display(STOP01);
    
    forward(1000, 150, 150);
    reverse(1000, 150, 150);
    leftSpin(800, 150);
    rightSpin(800, 150);
    
    pointCentre();
    displayClear();
    return;
  }

  // --- 2. Individual Obstacles -> Point & Nudge ---
  if (obsL) {
    Serial.println("DIAG: Left Obstacle");
    pointLeft();
    if (detectedRobot == _4TRONIX) tiltUp();
    else leftSpin(200, 150); // Small nudge for KeyeStudio
    delay(500);
    pointCentre();
    return;
  }

  if (obsR) {
    Serial.println("DIAG: Right Obstacle");
    pointRight();
    if (detectedRobot == _4TRONIX) tiltDown();
    else rightSpin(200, 150); // Small nudge for KeyeStudio
    delay(500);
    pointCentre();
    return;
  }

  // --- 3. Line Sensors -> Head movement ---
  // If Left is BLACK -> Look Right (standard behaviour in original KS Test)
  if (L == BLACK && R == WHITE) {
    Serial.println("DIAG: Line Left -> Look Right");
    pointRight();
    delay(500);
    pointCentre();
    return;
  }

  // If Right is BLACK -> Look Left
  if (R == BLACK && L == WHITE) {
    Serial.println("DIAG: Line Right -> Look Left");
    pointLeft();
    delay(500);
    pointCentre();
    return;
  }

  // If Centre is BLACK (KS) or Both are BLACK (4T) -> Nudge Forward
  bool lineForward = (C == BLACK || (detectedRobot == _4TRONIX && L == BLACK && R == BLACK));
  if (lineForward) {
    Serial.println("DIAG: Line Forward -> Nudge");
    forward(300, 150, 150);
    return;
  }

  delay(50);
}
