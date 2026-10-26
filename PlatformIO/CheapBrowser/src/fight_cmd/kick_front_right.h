void Kick_Front_Right(){

  float Angles[4][3] = {};
  
  SetInitialFightPositionSupportLeftLeg(Angles);

  // Begin Attack Move

  Angles[3][0] =  70.0;
  Angles[3][1] = -70.0;
  Angles[3][2] = 140.0;
 
  LegMove(Angles, 3);
  delay(250);

  
  for (int i = 6; i > 3; i--) {
    Angles[3][1] = -10.0 * i;
    Angles[3][2] =  20.0 * i;
 
    LegMove(Angles, 3);
    delay(100);
  }
 
  Angles[3][1] = -30.0;
  Angles[3][2] =  50.0;
 
  LegMove(Angles, 3);
  delay(100);


  Angles[3][1] = -20.0;
  Angles[3][2] =   0.0;
 
  LegMove(Angles, 3);
  delay(100);


  Angles[3][1] = -10.0;
 
  LegMove(Angles, 3);
  delay(100);


  Angles[3][1] = 0.0;
 
  LegMove(Angles, 3);
  delay(500);

  // End Attack Move 
  // Return to Normal
  ReturnToWalkingPositionSupportLeftLeg(Angles);
}

void cmd_Kick_Front_Right(RobotState* s, int, int)    { Kick_Front_Right(); }
