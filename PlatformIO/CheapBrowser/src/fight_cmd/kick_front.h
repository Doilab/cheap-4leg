void Kick_Front(){

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

  AngleMove(Angles);
  delay(500);


  Angles[3][0] = 70.0;

  AngleMove(Angles);
  delay(500);
  
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
  Angles[0][0] =  0.0;
  Angles[0][1] =  5.0;
  Angles[0][2] = 90.0;

  AngleMove(Angles);
  delay(500);


  Angles[3][0] =  0.0;
  Angles[3][2] = 90.0;
 
  AngleMove(Angles);
  delay(500);


  Angles[0][0] =  40.0;
  Angles[1][0] = -30.0;
  Angles[2][0] =  30.0;
  Angles[3][0] = -40.0;
 
  AngleMove(Angles);
  delay(500);

 
  Angles[0][2] = 100.0;
  Angles[1][2] = 100.0;
  Angles[2][2] = 100.0;
  Angles[3][2] = 100.0;
 
  AngleMove(Angles);
  delay(500);

}

void cmd_Kick_Front(RobotState* s, int, int)    { Kick_Front(); }
