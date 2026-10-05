#include "SerialLog.h"

void Kick_Left_Right(){

  float Angles[4][3] = {};
        Angles[0][0] =  0.0;
        Angles[0][1] =  5.0;
        Angles[0][2] = 90.0;
        Angles[1][0] =  0.0;
        Angles[1][1] =  5.0;
        Angles[1][2] = 90.0;
        Angles[2][0] =  0.0;
        Angles[2][1] =  5.0;
        Angles[2][2] = 90.0;
        Angles[3][0] =  0.0;
        Angles[3][1] =  5.0;
        Angles[3][2] = 90.0;

  char buf[64];

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

  Angles[3][0] = 70.0;

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

  Angles[0][0] = 70.0;
  Angles[0][2] =  0.0;

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

  Angles[0][0] = -70.0;

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);
  
  Angles[0][0] =  0.0;
  Angles[0][2] = 90.0;


  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

  Angles[3][0] =  0.0;
  Angles[3][2] = 90.0;


  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

  Angles[0][0] =  40.0;
  Angles[1][0] = -30.0;
  Angles[2][0] =  30.0;
  Angles[3][0] = -40.0;

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);
  
  Angles[0][2] = 100.0;
  Angles[1][2] = 100.0;
  Angles[2][2] = 100.0;
  Angles[3][2] = 100.0;

  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
  delay(500);

}

void cmd_Kick_Left_Right(RobotState* s, int, int)    { Kick_Left_Right(); }
