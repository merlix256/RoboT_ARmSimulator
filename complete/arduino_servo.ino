void setup() {
  Serial.begin(115200);
  Serial.println("Arduino started");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    
    // Parse comma-separated angles
    int angle_count = 0;
    float angles[3];
    char* token = strtok((char*)data.c_str(), ",");
    
    while (token != NULL && angle_count < 3) {
      angles[angle_count] = atof(token);
      angle_count++;
      token = strtok(NULL, ",");
    }
    
    if (angle_count == 3) {
      // Print only when we receive all three angles
      Serial.print("Angles updated: ");
      for (int i = 0; i < 3; i++) {
        Serial.print(angles[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}