// arduino_controller.ino
#include <Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Define servo pins
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;

// Variables for servo angles
float currentAngle1 = 90;
float currentAngle2 = 90;
float currentAngle3 = 90;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Attach servos to pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  // Set initial positions
  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
  servo3.write(currentAngle3);
  
  Serial.println("Robot arm initialized");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    // Check if it's a valid angle message
    if (input.startsWith("<") && input.endsWith(">")) {
      // Remove brackets
      input = input.substring(1, input.length() - 1);
      
      // Find commas
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      
      if (firstComma != -1 && secondComma != -1) {
        // Parse angles
        float angle1 = input.substring(0, firstComma).toFloat();
        float angle2 = input.substring(firstComma + 1, secondComma).toFloat();
        float angle3 = input.substring(secondComma + 1).toFloat();
        
        // Map angles to servo range (0-180)
        angle1 = constrain(angle1 + 90, 0, 180);
        angle2 = constrain(angle2 + 90, 0, 180);
        angle3 = constrain(angle3 + 90, 0, 180);
        
        // Move servos smoothly
        moveServoSmooth(1, currentAngle1, angle1);
        moveServoSmooth(2, currentAngle2, angle2);
        moveServoSmooth(3, currentAngle3, angle3);
        
        // Update current angles
        currentAngle1 = angle1;
        currentAngle2 = angle2;
        currentAngle3 = angle3;
        
        // Send confirmation
        Serial.print("Moved to angles: ");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.print(angle2);
        Serial.print(", ");
        Serial.println(angle3);
      }
    }
  }
}

// Function for smooth servo movement
void moveServoSmooth(int servoNum, float startAngle, float endAngle) {
  float step = 1.0;
  if (startAngle > endAngle) step = -1.0;
  
  for (float angle = startAngle; 
       step > 0 ? angle <= endAngle : angle >= endAngle; 
       angle += step) {
    switch(servoNum) {
      case 1: servo1.write(angle); break;
      case 2: servo2.write(angle); break;
      case 3: servo3.write(angle); break;
    }
    delay(15);  // Adjust this delay to control movement speed
  }
}