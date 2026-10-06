#include "SerialLog.h"

void Kick_Left_Right(){

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


  Angles[0][0] = 70.0;
  Angles[0][2] =  0.0;
 
  AngleMove(Angles);
  delay(500);


  Angles[0][0] = -70.0;
 
  AngleMove(Angles);
  delay(500);

 
  Angles[0][0] =  0.0;
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

void cmd_Kick_Left_Right(RobotState* s, int, int)    { Kick_Left_Right(); }
