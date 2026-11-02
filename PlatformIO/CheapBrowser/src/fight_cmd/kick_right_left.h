#include "SerialLog.h"

void Kick_Right_Left(RobotState *state) {

  float Angles[4][3];

  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      Angles[leg][j] = state->legs[leg].jointAngles[j];
    }
  }
  
  SetInitialFightPositionSupportLeftLeg(Angles);

  // Beginn Attack
  Angles[3][0] = -70.0;
  Angles[3][2] =  0.0;
 
  LegMove(Angles, 3);
  delay(500);


  Angles[3][0] = 70.0;
 
  LegMove(Angles, 3);
  delay(500);

  // Return to Normal
  ReturnToWalkingPositionSupportLeftLeg(Angles);
}

void cmd_Kick_Right_Left(RobotState* s, int, int)    { Kick_Right_Left(s); }
