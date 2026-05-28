//モーションを記述する
#ifndef INTERMITTENT_CRAWL_GAIT_H
#define INTERMITTENT_CRAWL_GAIT_H
//#include <math.h>
#include "kinematics.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
//---------------------------------------------
class IntermittentCrawlGait {
  public:
    float stride = 40; // 歩幅
    float f_height = 20; // 遊脚高さ
    glm::vec3 FB_vec[4];//歩容軌道の中心点．FootBase
    IntermittentCrawlGait() {
      // コンストラクタで必要な初期化を行う
    };
    void SetFootBaseDefault(void);
    glm::vec3 ICRectangleFootMotion(float phase, char flagFrontLeg);
    void Update(float phase, RobotState *state_out);
    void Update_Back(float phase, RobotState *state_out);
};
//---------------------------------------------

//---------------------------------------------
glm::vec3 IntermittentCrawlGait::ICRectangleFootMotion(float phase, char flagFrontLeg )
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
void IntermittentCrawlGait::SetFootBaseDefault(void)
{
  //間歇クロール歩容のモーション生成の関数
  float height = 80;//胴体高さ
  float dx_init = 10;//初期着地点をx方向に広げる幅
  float dy_init = 60;//初期着地点をy方向に広げる幅
  float base_x = 50;//脚付け根x座標（Body座標）
  float base_y = 50;//脚付け根y座標（Body座標）
  float COG[]={0,0};//重心の水平方向補正

  FB_vec[0].x = base_x+dx_init+COG[0];
  FB_vec[0].y = base_y+dy_init+COG[1];
  FB_vec[0].z = -height; 
  FB_vec[1].x = -base_x-dx_init+COG[0];
  FB_vec[1].y = base_y+dy_init+COG[1];
  FB_vec[1].z = -height;
  FB_vec[2].x = -base_x-dx_init+COG[0];
  FB_vec[2].y = -base_y-dy_init+COG[1];
  FB_vec[2].z = -height;
  FB_vec[3].x = base_x+dx_init+COG[0];
  FB_vec[3].y = -base_y-dy_init+COG[1];
  FB_vec[3].z = -height;

}

//---------------------------------------------
void IntermittentCrawlGait::Update(float phase, RobotState *state_out)
{
  //間歇クロール歩容のモーション生成の関数
 
  //float x,y,z;
  glm::vec3 FPos;
  char log_buffer[100];
  for(int leg=0;leg<4;leg++)
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

    glm::vec3 FPos2;
    FPos2 = FB_vec[leg] + FPos;//歩容軌道の中心点からの相対位置を足す．
//    Pos2.x = FB[leg][0]+FPos.x;
//    Pos2.y = FB[leg][1]+FPos.y;
//    Pos2.z = FB[leg][2]+FPos.z;

    SetFootPosIKBodyCoordinateToRobotState(leg, FPos2, state_out);

    sprintf(log_buffer, "ICrawl Update: leg=%d, phase=%.2f, x=%.1f, y=%.1f, z=%.1f", leg, leg_phase, FPos2.x, FPos2.y, FPos2.z);
    Serial.println(log_buffer);
  }
}
//---------------------------------------------
//後退歩行のモーション生成
void IntermittentCrawlGait::Update_Back(float phase, RobotState *state_out)
 {
  float r_phase = fmod(1-phase,1.0);//ICrawlのモードを逆順にする
  Update(r_phase, state_out);
} 
#endif