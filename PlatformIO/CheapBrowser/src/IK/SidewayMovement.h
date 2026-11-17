//モーションを記述する
#ifndef SIDEWAY_GAIT_H
#define SIDEWAY_GAIT_H
//#include <math.h>
#include "../movement/kinematics.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
//---------------------------------------------
class SidewayGait {
  public:
    float stride = 40; // 歩幅
    float f_height = 25; // 遊脚高さ
    glm::vec3 FB_vec[4];//歩容軌道の中心点．FootBase
    SidewayGait() {
      // コンストラクタで必要な初期化を行う
    };
    void SetFootBaseDefault(void);
    float Transition(float phase, float start, float length);
    glm::vec3 ICRectangleFootMotion(float phase, int Leg);
    void Update(float phase, RobotState *state_out);
    void Update_Back(float phase, RobotState *state_out);
};
//---------------------------------------------
float SidewayGait::Transition(float phase, float start, float length) 
{
  return (phase - start) / length;
}

//---------------------------------------------
glm::vec3 SidewayGait::ICRectangleFootMotion(float phase, int Leg )
{
  //float stride = 40;
  //float f_height = 25;
  float x = 0, y = 0, z = 0;

  bool flagFrontLeg = (Leg == 1) || (Leg == 0);
  bool flagLeft = (Leg == 1) || (Leg == 2);

  // x
  if (flagLeft) { 
    x = -20; 
  }
  else { 
    x = 30; // Front Side needs the Legs further out.
  }


  // y 
  if((phase<=0)||(phase>=1.0))
  {
      y = -0.5*stride;
  }
  else if((0<phase)&&(phase<=0.40))
  {
    float y_phase = phase;
    if(flagFrontLeg) { y_phase = phase - 0.2; }

    if(y_phase<=0.05)//[0, 0.05][-0.2, 0.05]
    {
      y = -0.5*stride;
    }
    else if(y_phase<=0.15)//[0.05, 0.15]
    {
      y = stride * (Transition(y_phase, 0.05, 0.10) -0.5);
    }
    else if(y_phase<=0.20)//[0.15, 0.4][0.15, 0.2]
    {
      y = 0.5*stride;
    }
  }
  else if((0.40<phase)&&(phase<0.5))//胴体推進1（足後ろに）
  {
      y = 0.5*stride * (1 - Transition(phase, 0.4, 0.10));
  }
  else if(phase<=0.9)//半周期後の姿勢で待機
  {
      y = 0;
  }
  else if(phase<=1.0)//胴体推進2（足後ろに）
  {
      y = -0.5*stride * Transition(phase, 0.9, 0.10);
  }


  // z
  if((phase<=0)||(0.40<phase))// On Ground
  {
    z = 0;
  }
  else //if((0<phase)&&(phase<=0.40))
  {
    float z_phase = phase;
    if(flagFrontLeg) { z_phase = phase - 0.2; }
    
    if ((z_phase < 0)||(0.2 < z_phase))
    { 
      z = 0;
    }
    else if(z_phase<=0.05)//[0, 0.05]
    {
      z = f_height * Transition(z_phase, 0, 0.05);
    }
    else if(z_phase<=0.15)//[0.05, 0.15]
    {
      z=f_height;
    }      
    else if(z_phase<=0.20)//[0.15, 0.20]
    {
      z = f_height * (1 - Transition(z_phase, 0.15, 0.05));
    }
  }

  glm::vec3 pos_out(x, y, z);

  return pos_out;
}
//---------------------------------------------
void SidewayGait::SetFootBaseDefault(void)
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
void SidewayGait::Update(float phase, RobotState *state_out)
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
    if(leg == 0) leg_phase = fmod(phase + 0.50, 1.0);//左半身
    if(leg == 2) leg_phase = fmod(phase + 0.00, 1.0);//右半身
    if(leg == 3) leg_phase = fmod(phase + 0.50, 1.0);//右半身

    FPos = ICRectangleFootMotion(leg_phase, leg);

    glm::vec3 FPos2;
    FPos2 = FB_vec[leg] + FPos;//歩容軌道の中心点からの相対位置を足す．
//    Pos2.x = FB[leg][0]+FPos.x;
//    Pos2.y = FB[leg][1]+FPos.y;
//    Pos2.z = FB[leg][2]+FPos.z;

    SetFootPosIKBodyCoordinateToRobotState(leg, FPos2, state_out);

    sprintf(log_buffer, "ICrawl Update: leg=%d, phase=%.2f, x=%.1f, y=%.1f, z=%.1f", leg, leg_phase, FPos2.x, FPos2.y, FPos2.z);
    logPrintln(log_buffer);
  }
}
//---------------------------------------------
//後退歩行のモーション生成
void SidewayGait::Update_Back(float phase, RobotState *state_out)
 {
  float r_phase = fmod(1-phase,1.0);//ICrawlのモードを逆順にする
  Update(r_phase, state_out);
} 
#endif
