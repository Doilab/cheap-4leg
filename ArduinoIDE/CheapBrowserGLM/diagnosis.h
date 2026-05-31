//テスト関数群
#ifndef DIAGNOSIS_H
#define DIAGNOSIS_H

#include "servo.h"
#include "kinematics.h"

//---------------------------------------------

void SetLegAngleArrayPattern(int num, float Angles_array_inout[4][3])
{
  //関節角度パターンセット
  if(num==0)
  {
    //すべてゼロ
    for(int i=0;i<4;i++)
    {
      Angles_array_inout[i][0]=0.0;
      Angles_array_inout[i][1]=0.0;
      Angles_array_inout[i][2]=0.0;
    }
  }
  else if(num==1)
  {
    for(int i=0;i<4;i++)
    {
      Angles_array_inout[i][0]=0.0;
      Angles_array_inout[i][1]=0.0;
      Angles_array_inout[i][2]=90.0;
    }
  }
  else if(num==2)
  {
    for(int i=0;i<4;i++)
    {
      Angles_array_inout[i][0]=0.0;
      Angles_array_inout[i][1]=-45.0;
      Angles_array_inout[i][2]=90.0;
    }
  }
  else if(num==3)
  {
    for(int i=0;i<4;i++)
    {
      Angles_array_inout[i][0]=20.0;
      Angles_array_inout[i][1]=0.0;
      Angles_array_inout[i][2]=90.0;
    }
  }
  else if(num==4)
  {
    for(int i=0;i<4;i++)
    {
      Angles_array_inout[i][0]=0.0;
      Angles_array_inout[i][1]=51.2;
      Angles_array_inout[i][2]=25.0;
    }
  }
  return;
}


void PWM_test(void)// モータチェック用
{
  char buf[64];
  Serial.println("PWM_test()1 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("PWM_test()--quit");
      return;
    }
    if(str1=="1")
    {
      int value = 300;//PWM
      int ch =7;//モータ１個テスト用
      sprintf(buf,"PWM ch[%d] value[%d]",ch,value);
      Serial.println(buf);
      pwm.setPWM(ch, 0, value);//生のPWM設定．モータ１個テスト用
    }
 
  }
}
//---------------------------------------------
void moveServo_test(void)
{
  //サーボ角での駆動テスト
  char buf[64];
  Serial.println("moveServo_test()1 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("moveServo_test()--quit");
      return;
    }
    if(str1=="1")
    {
      int value = 10;//サーボ角
      int ch =7;//モータ１個テスト用
      sprintf(buf,"moveServo ch[%d] value[%d]",ch,value);
      Serial.println(buf);
      moveServo(ch, value);//角度入力（サーボ角）．モータ１個テスト用
    }
 
  }
}
//---------------------------------------------
void moveLegWithOffset_test(void)
{
  //Leg角＋配列での駆動テスト
  char buf[64];
  float Angles[4][3];
  Serial.println("moveLegWithOffset_test()0,1,2,3,4 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("SetAnglesFromArray_test()--quit");
      return;
    }
    else if(str1=="0")
    {
      sprintf(buf,"SetAnglesFromArray 0");
      Serial.println(buf);
      SetLegAngleArrayPattern(0, Angles);
    }
    else if(str1=="1")
    {
      sprintf(buf,"SetAnglesFromArray 1");
      Serial.println(buf);
      SetLegAngleArrayPattern(1, Angles);
    }
    else if(str1=="2")
    {
      sprintf(buf,"SetAnglesFromArray 2");
      Serial.println(buf);
      SetLegAngleArrayPattern(2, Angles);
    }
    else if(str1=="3")
    {
      sprintf(buf,"SetAnglesFromArray 3");
      Serial.println(buf);
      SetLegAngleArrayPattern(3, Angles);
    }
    else if(str1=="4")
    {
      sprintf(buf,"SetAnglesFromArray 4");
      Serial.println(buf);
      SetLegAngleArrayPattern(4, Angles);
    }
    else
    {
      return;
    }
    for(int leg=0;leg<4;leg++)
    {
      float x=Angles[leg][0];
      float y=Angles[leg][1];
      float z=Angles[leg][2];
      moveLegWithOffset(leg, x,y,z);
      sprintf(buf,"Leg%d angles (%.1f, %.1f, %.1f)", leg, x,y,z);
      Serial.println(buf);
    }
 
  }
}

//---------------------------------------------
void SetFootPosIKLegCoordinate_test(void)
{
  //脚座標で逆運動学のテスト
  char buf[64];
  ///float AnglesIK[4][3];
  float x,y,z;
  Serial.println("SetFootPosIKLegCoordinate_test()1 or 2 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("SetFootPosIKLegCoordinate_test()--quit");
      return;
    }
    if(str1=="1")
    {
       x=90;
       y=0;
       z=-80;
      
      sprintf(buf,"SetPosIKLegCoordinate (%.1f, %.1f, %.1f )",x,y,z);
      Serial.println(buf);
    }
    else if(str1=="2")
    {
       x=90;
       y=10;
       z=-70;
      
      sprintf(buf,"SetPosIKLegCoordinate (%.1f, %.1f, %.1f )",x,y,z);
      Serial.println(buf);
    }
    else
    {
      return;
    }


    glm::vec3 pos_leg_coordinate(x,y,z);
    glm::vec3 angles_vec3;
    angles_vec3 = SetFootPosIKLegCoordinateToVec3(pos_leg_coordinate);
  
    RobotState state;
    for (int leg = 0; leg < 4; ++leg) {
      state.legs[leg].jointAngles = angles_vec3;
      moveLegWithOffset(leg, state.legs[leg].jointAngles[0], state.legs[leg].jointAngles[1], state.legs[leg].jointAngles[2]);
    }
    //SetAnglesFromState(state);
  }
}
//---------------------------------------------
void SetFootPosIKBodyCoordinate_test(void)
{
  //胴体座標で逆運動学のテスト
  char buf[64];
  //float AnglesIK[4][3];
  float FootPos[4][3];
  glm::vec3 FP[4];//FootPos

  double x,y,z;
  Serial.println("SetFootPosIKBodyCoordinate_test() 1 or 2 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("SetFootPosIKBodyCoordinate_test()--quit");
      return;
    }
    if(str1=="1")
    {
       x=140;
       y=50;
       z=-80;
       FP[0]=glm::vec3(x,y,z);
       FP[1]=glm::vec3(-x,y,z);
       FP[2]=glm::vec3(-x,-y,z);
       FP[3]=glm::vec3(x,-y,z);
      //  FootPos[0][0]=x;
      //  FootPos[0][1]=y; 
      //  FootPos[0][2]=z;
      //  FootPos[1][0]=-x;
      //  FootPos[1][1]=y; 
      //  FootPos[1][2]=z;
      //  FootPos[2][0]=-x;
      //  FootPos[2][1]=-y; 
      //  FootPos[2][2]=z;
      //  FootPos[3][0]=x;
      //  FootPos[3][1]=-y; 
      //  FootPos[3][2]=z;
    }
    else if(str1=="2")
    {
       x=50;
       y=140;
       z=-50;
        FP[0]=glm::vec3(x,y,z);
        FP[1]=glm::vec3(-x,y,z);
        FP[2]=glm::vec3(-x,-y,z);
        FP[3]=glm::vec3(x,-y,z);
      //  FootPos[0][0]=x;
      //  FootPos[0][1]=y; 
      //  FootPos[0][2]=z;
      //  FootPos[1][0]=-x;
      //  FootPos[1][1]=y; 
      //  FootPos[1][2]=z;
      //  FootPos[2][0]=-x;
      //  FootPos[2][1]=-y; 
      //  FootPos[2][2]=z;
      //  FootPos[3][0]=x;
      //  FootPos[3][1]=-y; 
      //  FootPos[3][2]=z;
    }
    else   
    {
      return;
    }
      sprintf(buf,"Body coordinate (%.0f, %.0f, %.0f)",x,y,z);
      Serial.println(buf);
      RobotState state;
      // for (int i = 0; i < 4; ++i) {
      //   FP[i] = glm::vec3(FootPos[i][0], FootPos[i][1], FootPos[i][2]);
      // }
      // SetFootPosIKBodyCoordinateToArray(0,FootPos[0][0],FootPos[0][1],FootPos[0][2], AnglesIK);
      // SetFootPosIKBodyCoordinateToArray(1,FootPos[1][0],FootPos[1][1],FootPos[1][2], AnglesIK);
      // SetFootPosIKBodyCoordinateToArray(2,FootPos[2][0],FootPos[2][1],FootPos[2][2], AnglesIK);
      // SetFootPosIKBodyCoordinateToArray(3,FootPos[3][0],FootPos[3][1],FootPos[3][2], AnglesIK);
      // SetAnglesFromArray(AnglesIK);
      for (int leg = 0; leg < 4; ++leg) {
        SetFootPosIKBodyCoordinateToRobotState(leg, FP[leg], &state);
        moveLegWithOffset(leg, state.legs[leg].jointAngles[0], state.legs[leg].jointAngles[1], state.legs[leg].jointAngles[2]);
      }
  }
}
//----------------------------------------------
#endif