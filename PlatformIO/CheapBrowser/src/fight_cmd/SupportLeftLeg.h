void SetInitialFightPositionSupportLeftLeg(float Angles[4][3]) {
  
  // Set Leg 0 in support Position
  // Using Leg 1 for stable Position
  float saveLeg1initpos = Angles[1][0];
  Angles[1][0] =  -100.0;
  LegMove(Angles, 1);
  delay(500);

  // Lift Tip 
  Angles[0][2] = 140; 
  LegMove(Angles, 0);
  delay(500);

  // Move Leg down
  Angles[0][1] = 90; 
  LegMove(Angles, 0);
  delay(500);
  
  // Position Tip 
  Angles[0][0] = -70.0;
  Angles[0][2] = 100; 
  LegMove(Angles, 0);
  delay(500);

  // Return Leg 0 To init Position
  Angles[1][0] =  saveLeg1initpos;
  LegMove(Angles, 1);
  delay(500);

}

void ReturnToWalkingPositionSupportLeftLeg(float Angles[4][3]) {
  // Leg 3 stable
  Angles[3][0] =  0.0;
  Angles[3][1] =  5.0;
  Angles[3][2] = 100.0;
 
  LegMove(Angles, 3);
  delay(250);

  // Return Leg 0 in normal Position
  // Using Leg 1 for stable Position
  float saveLeg1initpos = Angles[1][0];
  Angles[1][0] =  -100.0;
  LegMove(Angles, 1);
  delay(500);

  // Lift Tip 
  Angles[0][2] = 140; 
  LegMove(Angles, 0);
  delay(500);

  // Move Leg up
  Angles[0][1] = 5; 
  LegMove(Angles, 0);
  delay(500);

  // Normal position
  Angles[0][0] = 40.0;
  Angles[0][2] = 90.0;
 
  LegMove(Angles, 0);
  delay(500);

  // Return to Walk Position

  Angles[1][0] =  saveLeg1initpos;
  LegMove(Angles, 1);

  delay(500);
 
}
