#ifndef MANUAL_MOVEMENT_H
#define MANUAL_MOVEMENT_H

#include <Arduino.h>
#include "../movement/servo.h"

bool parseThreeFloats(const String& input, float& a, float& b, float& c) {
  int idx1 = input.indexOf(' ');
  if (idx1 < 0) return false;
  
  int idx2 = input.indexOf(' ', idx1 + 1);
  if (idx2 < 0) return false;

  a = input.substring(0, idx1).toFloat();
  b = input.substring(idx1 + 1, idx2).toFloat();
  c = input.substring(idx2 + 1).toFloat();
  return true;
}

void ManualMove() {
  float Angles[4][3] = {};
  char buf[64];

  Serial.println("ManualMove: Enter Angle like ('x.x y.y z.z'), Enter = use the last numbers, 'q' to exit");

  while(1) {
    for (int i = 0; i < 4; i++) {
      sprintf(buf, "Leg %d > ", i);
      Serial.println(buf);

      // Wait till input
      while (Serial.available() <= 0) { ; }
      String str = Serial.readString();
      str.trim();

      if (str == "q") {
        Serial.println("ManualMove: exit");
        return;
      }
      
      if (str.length() == 0) {
        continue; // Use last floats
      }

      float a, b, c;
      if (parseThreeFloats(str, a, b, c)) {
        Angles[i][0] = a;
        Angles[i][1] = b;
        Angles[i][2] = c;
      } else {
        Serial.println("Invaild Input, please retry");
        i--; // decrease counter to retry
      }
    }

    // Move all 4 Legs
    Serial.println();
    for (int leg = 0; leg < 4; leg++) {
      float x = Angles[leg][0];
      float y = Angles[leg][1];
      float z = Angles[leg][2];
      moveLegWithOffset(leg, x, y, z);
      sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
      Serial.println(buf);
    }
    Serial.println();
  }
}

#endif
