#include "SerialLog.h"

void Kick_Right_Left(){
  
  float Angles[4][3] = {};
  
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

void cmd_Kick_Right_Left(RobotState* s, int, int)    { Kick_Right_Left(); }
