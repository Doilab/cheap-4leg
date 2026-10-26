void Kick_Front_Left(){

  float Angles[4][3] = {};
  
  SetInitialFightPositionSupportRightLeg(Angles);

  // Begin Attack Move

  Angles[0][0] = -70.0;
  Angles[0][1] = -70.0;
  Angles[0][2] = 140.0;
 
  LegMove(Angles, 0);
  delay(250);

  
  for (int i = 6; i > 3; i--) {
    Angles[0][1] = -10.0 * i;
    Angles[0][2] =  20.0 * i;
 
    LegMove(Angles, 0);
    delay(100);
  }
 
  Angles[0][1] = -30.0;
  Angles[0][2] =  50.0;
 
  LegMove(Angles, 0);
  delay(100);


  Angles[0][1] = -20.0;
  Angles[0][2] =   0.0;
 
  LegMove(Angles, 0);
  delay(100);


  Angles[0][1] = -10.0;
 
  LegMove(Angles, 0);
  delay(100);


  Angles[0][1] = 0.0;
 
  LegMove(Angles, 0);
  delay(500);

  // End Attack Move 
  // Return to Normal
  ReturnToWalkingPositionSupportRightLeg(Angles);
}

void cmd_Kick_Front_Left(RobotState* s, int, int)    { Kick_Front_Left(); }
