//モーションを記述する
#ifndef MOTION_H
#define MOTION_H
//#include <math.h>
#include "kinematics.h"
#include <glm/gtc/matrix_transform.hpp>
//---------------------------------------------

//---------------------------------------------
void SetInitialPose(RobotState *state_out)
{
  //初期姿勢にする関数
  //RobotState構造体初期化
  for (int leg = 0; leg < 4; ++leg) {
    state_out->legs[leg].jointAngles[0] = 0.0;
    state_out->legs[leg].jointAngles[1] = 0.0;
    state_out->legs[leg].jointAngles[2] = 0.0;
    //脚先位置は関節角度から計算して入れる．
    //↓未実装
    //float x, y, z;
    // calcFK_Leg(state_out->legs[leg].jointAngles[0], state_out->legs[leg].jointAngles[1], state_out->legs[leg].jointAngles[2], &x, &y, &z);
    // state_out->legs[leg].footPos = glm::vec3(x, y, z);
  }
}
//---------------------------------------------

//---------------------------------------------
glm::vec3 ICRectangleFootMotion(float phase, char flagFrontLeg )
{
  //脚先  １個分を長方形状に動かす動作生成の関数
  //間歇クロール専用
  float stride = 40;//歩幅
  float f_height = 20;//遊脚高さ
  float x,y,z;
  float dp, ratio;
  
  if((phase<=0)||(phase>=1.0))
  {
    //front/rear関係なく同じ位置
      x=(-0.5*stride);
      y=0;
      z=0;
  }
  else if((phase>0)&&(phase<=0.40))
  {
    if(flagFrontLeg !=1)//後脚の場合
    {
        if(phase<=0.05)//足上げ
        {
          dp=phase;
          ratio=dp/0.05;
          z=f_height*ratio;
          x=-0.5*stride;
          y=0;
        }
        else if(phase<=0.15)//足復帰
        {
          dp=phase-0.05;
          ratio=dp/0.10;
          x=stride*ratio-0.5*stride;
          y=0;
          z=f_height;
        }
        else if(phase<=0.20)//足おろし
        {
          dp=phase-0.15;
          ratio=dp/0.05;
          x=0.5*stride;
          y=0;
          z=f_height*(1-ratio);
        }
        else if(phase<=0.4)//待機
        {
          x=0.5*stride;
          y=0;
          z=0;
        }
    }
    else if(flagFrontLeg ==1)//前脚の場合
    {
        if(phase<0.20)//待機
        {
          x=(-0.5*stride);
          y=0;
          z=0;
        }
        else if(phase<=0.25)//足上げ
        {
          dp=phase-0.20;
          ratio=dp/0.05;
          z=f_height*ratio;
          x=-0.5*stride;
          y=0;
        }
        else if(phase<=0.35)//足復帰
        {
          dp=phase-0.25;
          ratio=dp/0.10;
          x=stride*ratio-0.5*stride;
          y=0;
          z=f_height;
        }
        else if(phase<=0.40)//足おろし
        {
          dp=phase-0.35;
          ratio=dp/0.05;
          x=0.5*stride;
          y=0;
          z=f_height*(1-ratio);
        }
    }
  }
  else if((phase>0.40)&&(phase<0.5))//胴体推進1（足後ろに）
  {
    dp=phase-0.4;
    ratio=dp/0.10;
    x=0.5*stride - stride*0.5*(ratio);
    y=0;
    z=0;
  }
  else if(phase<=0.9)//半周期後の姿勢で待機
  {
    //front/rear関係なく同じ位置
      x=0;
      y=0;
      z=0;
  }
  else if(phase<=1.0)//胴体推進2（足後ろに）
  {
    dp=phase-0.9;
    ratio=dp/0.10;
    x= (-stride*0.5*(ratio));
    y=0;
    z=0;
  }
 
  glm::vec3 pos_out(x, y, z);

  return pos_out;
}
//---------------------------------------------
void ICrawl(float phase, RobotState *state_out)
{
  //間歇クロール歩容のモーション生成の関数
  float height = 80;//胴体高さ
  float dx_init = 10;//初期着地点をx方向に広げる幅
  float dy_init = 60;//初期着地点をy方向に広げる幅
  float base_x = 50;//脚付け根x座標（Body座標）
  float base_y = 50;//脚付け根y座標（Body座標）
  float COG[]={0,0};//重心の水平方向補正
  int leg;//動かす箇所

  //float FootPos[4][3];//脚先位置の配列

  //歩容軌道の中心点
  float FB[4][3];
  FB[0][0]=base_x+dx_init+COG[0];
  FB[0][1]=base_y+dy_init+COG[1];
  FB[0][2]=-height;
  FB[1][0]=-base_x-dx_init+COG[0];
  FB[1][1]=base_y+dy_init+COG[1];
  FB[1][2]=-height;
  FB[2][0]=-base_x-dx_init+COG[0];
  FB[2][1]=-base_y-dy_init+COG[1];
  FB[2][2]=-height;
  FB[3][0]=base_x+dx_init+COG[0];
  FB[3][1]=-base_y-dy_init+COG[1];
  FB[3][2]=-height;

  float x,y,z;
  glm::vec3 FPos;
  char log_buffer[100];
  for(leg=0;leg<4;leg++)
  {
    float leg_phase = 0;
    // 左右半身で各脚の位相をずらす．
    if(leg == 1) leg_phase = fmod(phase + 0.00, 1.0);//左半身 
    if(leg == 0) leg_phase = fmod(phase + 0.00, 1.0);//左半身
    if(leg == 2) leg_phase = fmod(phase + 0.50, 1.0);//右半身
    if(leg == 3) leg_phase = fmod(phase + 0.50, 1.0);//右半身
    //脚の動きの生成
    if((leg == 1)||(leg==2)){
      FPos = ICRectangleFootMotion(leg_phase, 0);//RearLeg
    }
    if((leg == 0)||(leg==3)){
      FPos = ICRectangleFootMotion(leg_phase, 1);//FrontLeg
    }

    glm::vec3 Pos2;
    Pos2.x = FB[leg][0]+FPos.x;
    Pos2.y = FB[leg][1]+FPos.y;
    Pos2.z = FB[leg][2]+FPos.z;

    SetFootPosIKBodyCoordinateToRobotState(leg,Pos2, state_out);

    sprintf(log_buffer, "ICrawl2: leg=%d, phase=%.2f, x=%.1f, y=%.1f, z=%.1f", leg, leg_phase, Pos2.x, Pos2.y, Pos2.z);
    Serial.println(log_buffer);
  }
}
//---------------------------------------------
//後退歩行のモーション生成
void ICrawl_Back(float phase, RobotState *state_out)
 {
  float r_phase = fmod(1-phase,1.0);//ICrawlのモードを逆順にする
  ICrawl(r_phase, state_out);
} 
#endif