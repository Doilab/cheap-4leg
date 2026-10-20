void Kick_Front(){

  float Angles[4][3] = {};
  
  SetInitialFightPosition(Angles);
  MoveLeg3InSupportPosition(Angles);
  
  // Begin Attack Move

  Angles[0][0] = -70.0;
  Angles[0][1] = -70.0;
  Angles[0][2] = 140.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -60.0;
  Angles[0][2] = 120.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -50.0;
  Angles[0][2] = 100.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -40.0;
  Angles[0][2] =  80.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -30.0;
  Angles[0][2] =  50.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -20.0;
  Angles[0][2] =   0.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = -10.0;
 
  AngleMove(Angles);
  delay(250);


  Angles[0][1] = 0.0;
 
  AngleMove(Angles);
  delay(250);

  // End Attack Move 
  // Return to Normal
  MoveLeg3InStandartPosition(Angles);
  ReturnToWalkingPosition(Angles);
}

void cmd_Kick_Front(RobotState* s, int, int)    { Kick_Front(); }
