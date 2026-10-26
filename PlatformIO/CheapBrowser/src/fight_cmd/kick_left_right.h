#include "SerialLog.h"

void Kick_Left_Right(){
  
  float Angles[4][3] = {};
  
  SetInitialFightPositionSupportRightLeg(Angles);

  // Beginn Attack
  Angles[0][0] = 70.0;
  Angles[0][2] =  0.0;
 
  LegMove(Angles, 0);
  delay(500);


  Angles[0][0] = -70.0;
 
  LegMove(Angles, 0);
  delay(500);

  // Return to Normal
  ReturnToWalkingPositionSupportRightLeg(Angles);
}

void cmd_Kick_Left_Right(RobotState* s, int, int)    { Kick_Left_Right(); }
