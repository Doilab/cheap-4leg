void SetInitialFightPositionSupportRightLeg(float Angles[4][3]) {
  
  // Set Leg 3 in support Position
  // Using Leg 2 for stable Position
  float saveLeg2initpos = Angles[2][0];
  Angles[2][0] =  100.0;
  LegMove(Angles, 2);
  delay(500);

  // Lift Tip 
  Angles[3][2] = 140; 
  LegMove(Angles, 3);
  delay(500);

  // Move Leg down
  Angles[3][1] = 90; 
  LegMove(Angles, 3);
  delay(500);
  
  // Position Tip 
  Angles[3][0] = 70.0;
  Angles[3][2] = 100; 
  LegMove(Angles, 3);
  delay(500);

  // Return Leg 0 To init Position
  Angles[2][0] =  saveLeg2initpos;
  LegMove(Angles, 2);
  delay(500);

}

void ReturnToWalkingPositionSupportRightLeg(float Angles[4][3]) {
  // Leg 0 stable
  Angles[0][0] =  0.0;
  Angles[0][1] =  5.0;
  Angles[0][2] = 100.0;
 
  LegMove(Angles, 0);
  delay(250);

  // Return Leg 0 in normal Position
  // Using Leg 2 for stable Position
  float saveLeg2initpos = Angles[2][0];
  Angles[2][0] =  100.0;
  LegMove(Angles, 2);
  delay(500);


  // Lift Tip 
  Angles[3][2] = 140; 
  LegMove(Angles, 3);
  delay(500);

  // Move Leg up
  Angles[3][1] = 5; 
  LegMove(Angles, 3);
  delay(500);

  // Normal position
  Angles[3][0] = -40.0;
  Angles[3][2] = 90.0;
 
  LegMove(Angles, 3);
  delay(500);

  // Return to Walk Position

  Angles[2][0] =  saveLeg2initpos;
  LegMove(Angles, 2);
  delay(500);
 
}
