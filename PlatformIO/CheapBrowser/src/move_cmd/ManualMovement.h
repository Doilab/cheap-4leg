#ifndef MANUAL_MOVEMENT_H
#define MANUAL_MOVEMENT_H

#include <Arduino.h>
#include "../movement/servo.h"
#include "../SerialLog.h"

void drawTable(float Angles[4][3], int curRow, int curCol) {
  Serial.println("\n=== Manual Movement Editor ===");
  Serial.println("       Th1      Th2      Th3");
  Serial.println("  +--------+--------+--------+");

  for (int leg = 0; leg < 4; leg++) {
    char buf[80];
    Serial.printf("L%d|", leg);
    for (int joint = 0; joint < 3; joint++) {
      if (leg == curRow && joint == curCol) {
        Serial.printf(">%5.1f< |", Angles[leg][joint]);
      } else {
        Serial.printf(" %5.1f  |", Angles[leg][joint]);
      }
    }
    Serial.println();
    Serial.println("  +--------+--------+--------+");
  }
  Serial.println("W/S: Up/Down  A/D: Left/Right  Enter: Edit  J: Execute  Q: Exit");
}

void ManualMove() {
  float Angles[4][3] = {};
  int curRow = 0;
  int curCol = 0;

  // Initiale Winkel aus aktuellem Roboter-Zustand übernehmen wäre hier möglich
  // vorerst 0.0

  drawTable(Angles, curRow, curCol);

  while (1) {
    if (Serial.available() <= 0) continue;

    String input = Serial.readString();
    input.trim();
    char key = input.length() > 0 ? tolower(input[0]) : 0;

    if (key == 'q') {
      logPrintln("ManualMove: Exit");
      return;
    }
    else if (key == 'w') curRow = (curRow + 3) % 4;  // Up
    else if (key == 's') curRow = (curRow + 1) % 4;  // Down
    else if (key == 'a') curCol = (curCol + 2) % 3;  // Left
    else if (key == 'd') curCol = (curCol + 1) % 3;  // Right
    else if (key == 'j') {                           // Execute
      AngleMove(Angles);
    }
    else if (input.length() == 0 || key == '\n' || key == '\r') {
      // Enter: Value edit
      char prompt[40];
      sprintf(prompt, "Leg%d Th%d [%.1f] > ", curRow, curCol, Angles[curRow][curCol]);
      Serial.print(prompt);

      while (Serial.available() <= 0) { ; }
      String val = Serial.readString();
      val.trim();

      if (val.length() > 0) {
        float newVal = val.toFloat();
        Angles[curRow][curCol] = newVal;
      }
      // emthy → save old Value
    }

    drawTable(Angles, curRow, curCol);
  }
}

#endif
