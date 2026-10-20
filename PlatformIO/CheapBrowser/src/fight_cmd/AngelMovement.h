void AngleMove(float Angles[4][3]) {
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
}

void SetInitialFightPosition(float Angles[4][3]) {
  Angles[0][0] =  0.0; Angles[0][1] =  5.0; Angles[0][2] = 90.0;
  Angles[1][0] =  0.0; Angles[1][1] =  5.0; Angles[1][2] = 90.0;
  Angles[2][0] =  0.0; Angles[2][1] =  5.0; Angles[2][2] = 90.0;
  Angles[3][0] =  0.0; Angles[3][1] =  5.0; Angles[3][2] = 90.0;

  AngleMove(Angles);
  delay(500);
}

void MoveLeg3InSupportPosition(float Angles[4][3]) {
  // Set Leg 3 in support Position
  
  // Using Leg 0 for stable Position
  Angles[0][0] =  -70.0;
  Angles[0][1] = 5.0;
  Angles[0][2] = 90.0;
  AngleMove(Angles);
  delay(500);
  

  Angles[3][0] = 70.0;
  AngleMove(Angles);
  delay(500);

  // Lift Tip 
  Angles[3][2] = 140; 
  AngleMove(Angles);
  delay(500);

  // Move Leg down
  Angles[3][1] = 90; 
  AngleMove(Angles);
  delay(500);
  
  // Position Tip 
  Angles[3][2] = 100; 
  AngleMove(Angles);
  delay(500);

  // Return Leg 0 To init Position
  Angles[0][0] =  0.0;
  AngleMove(Angles);
  delay(500);
}

void MoveLeg3InStandartPosition(float Angles[4][3]) {
  // Return Leg 3 in normal Position

  Angles[0][0] =  -70.0;
  Angles[0][1] = 5.0;
  Angles[0][2] = 90.0;
  AngleMove(Angles);
  delay(500);

  // Lift Tip 
  Angles[3][2] = 140; 
  AngleMove(Angles);
  delay(500);

  // Move Leg up
  Angles[3][1] = 5; 
  AngleMove(Angles);
  delay(500);

  // Normal position
  Angles[3][0] =  0.0;
  Angles[3][2] = 90.0;
 
  AngleMove(Angles);
  delay(500);
}

void ReturnToWalkingPosition(float Angles[4][3]) {
  // Return to Walk Position

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
