void StandUp() {
  float Angles[4][3] = {};
  
  Angles[0][0] =  0.0; Angles[0][1] =  -90.0; Angles[0][2] = 160.0;
  Angles[1][0] =  0.0; Angles[1][1] =  -90.0; Angles[1][2] = 160.0;
  Angles[2][0] =  0.0; Angles[2][1] =  -90.0; Angles[2][2] = 160.0;
  Angles[3][0] =  0.0; Angles[3][1] =  -90.0; Angles[3][2] = 160.0;

for (int i = 1; i <= 3; i++) {
    Angles[0][1] = -90.0 + i * 10;
    Angles[1][1] = -90.0 + i * 10;
    Angles[2][1] = -90.0 + i * 10;
    Angles[3][1] = -90.0 + i * 10;
 
    AngleMove(Angles);
    delay(750);
  }
for (int i = 1; i <= 6; i++) {
    Angles[0][1] = -60.0 + i * 10;
    Angles[1][1] = -60.0 + i * 10;
    Angles[2][1] = -60.0 + i * 10;
    Angles[3][1] = -60.0 + i * 10;
    
    Angles[0][2] = 160.0 - i * 5;
    Angles[1][2] = 160.0 - i * 5;
    Angles[2][2] = 160.0 - i * 5;
    Angles[3][2] = 160.0 - i * 5;

    AngleMove(Angles);
    delay(1000);
  }

  delay(2000);
}
void cmd_stand_up(RobotState* s, int, int)    { StandUp(); }
