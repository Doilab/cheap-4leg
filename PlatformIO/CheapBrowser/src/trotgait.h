//モーションを記述する
#ifndef TROTGAIT_H
#define TROTGAIT_H
#include "kinematics.h"
#include <glm/gtc/matrix_transform.hpp>
//---------------------------------------------
class TrotGait
{
  //トロット歩容のクラス
  public:
    glm::vec3 FB_vec[4];//歩容軌道の中心点．FootBase
    float stride = 30;//歩幅

    void Update(float phase, int RotateMode, RobotState *state_out);

    glm::vec3 RectangleFootMotion(float phase);
    void SetFootBaseDefault(void);
    void SetFootBase(RobotState *state_in);
};

//---------------------------------------------

//---------------------------------------------
glm::vec3 TrotGait::RectangleFootMotion(float phase)
{
  //脚先  １個分を長方形状に動かす動作生成の関数
  //float stride = 30;//歩幅
  float f_height = 20;//遊脚高さ
  float x,y,z;
  float dp, ratio;
  
  if((phase<=0)||(phase>=1.0))
  {
      x=(-0.5*stride);
      y=0;
      z=0;
  }
  else if((phase>0)&&(phase<=0.50))
  {
    if(phase<=0.1)//足上げ
    {
      dp=phase;
      ratio=dp/0.1;
      z=f_height*ratio;
      x=-0.5*stride;
      y=0;
    }
    else if(phase<=0.3)//足復帰
    {
      dp=phase-0.1;
      ratio=dp/0.20;
      x=stride*ratio-0.5*stride;
      y=0;
      z=f_height;
    }
    else if(phase<=0.4)//足おろし
    {
      dp=phase-0.3;
      ratio=dp/0.1;
      x=0.5*stride;
      y=0;
      z=f_height*(1-ratio);
    }
    else if(phase<=0.5)//待機
    {
      x=0.5*stride;
      y=0;
      z=0;
    }
  }
  else if((phase>0.50)&&(phase<1.0))//胴体推進（足後ろに下げる）
  {
    dp=phase-0.5;
    ratio=dp/0.50;
    x=0.5*stride - stride*(ratio);
    y=0;
    z=0;
  }

  glm::vec3 pos_out(x, y, z);
  return pos_out;
}
//---------------------------------------------
void TrotGait::SetFootBaseDefault(void)
{
  //歩容軌道の中心点．FootBaseを初期値にする関数
  float height = 80;//胴体高さ
  float dx_init = 10;//初期着地点をx方向に広げる幅
  float dy_init = 60;//初期着地点をy方向に広げる幅
  float base_x = 50;//脚付け根x座標（Body座標）
  float base_y = 50;//脚付け根y座標（Body座標）

 //歩容軌道の中心点．FootBase
  FB_vec[0].x=base_x+dx_init;
  FB_vec[0].y=base_y+dy_init;
  FB_vec[0].z=-height;
  FB_vec[1].x=-base_x-dx_init;
  FB_vec[1].y=base_y+dy_init;
  FB_vec[1].z=-height;
  FB_vec[2].x=-base_x-dx_init;
  FB_vec[2].y=-base_y-dy_init;
  FB_vec[2].z=-height;
  FB_vec[3].x=base_x+dx_init;
  FB_vec[3].y=-base_y-dy_init;
  FB_vec[3].z=-height;

  for(int leg=0;leg<4;leg++)
  {
    String log_buffer = "SetFootBase: leg=" + String(leg) + ", x=" + String(FB_vec[leg].x) + ", y=" + String(FB_vec[leg].y) + ", z=" + String(FB_vec[leg].z);
    Serial.println(log_buffer);
  }
}
//---------------------------------------------
void TrotGait::SetFootBase(RobotState *state_in)
{
  //歩容軌道の中心点．FootBaseをRobotStateからセットする関数
  for(int leg=0;leg<4;leg++)
  {
    FB_vec[leg] = state_in->legs[leg].footPos;
  }

  for(int leg=0;leg<4;leg++)
  {
    String log_buffer = "SetFootBase: leg=" + String(leg) + ", x=" + String(FB_vec[leg].x) + ", y=" + String(FB_vec[leg].y) + ", z=" + String(FB_vec[leg].z);
    Serial.println(log_buffer);
  }
}
//---------------------------------------------
void TrotGait::Update(float phase, int RotateMode, RobotState *state_out)
{
  //トロット歩容のモーション生成の関数

  char log_buffer[100];
  for(int leg=0;leg<4;leg++)
  {
    float leg_phase = 0;
    // 各脚の位相をずらす．
    if(leg == 0) leg_phase = fmod(phase + 0.00, 1.0);//
    if(leg == 2) leg_phase = fmod(phase + 0.00, 1.0);//
    if(leg == 1) leg_phase = fmod(phase + 0.50, 1.0);// 
    if(leg == 3) leg_phase = fmod(phase + 0.50, 1.0);//
    //脚の動きの生成
    glm::vec3 FPos = RectangleFootMotion(leg_phase);
    if(leg==0)FPos.x +=0.5*stride;//0.5歩幅を前にずらす．スタート地点が現在地になるように
    if(leg==2)FPos.x +=0.5*stride;//0.5歩幅を前にずらす
    if(leg==1)FPos.x -=0.5*stride;//0.5歩幅を後ろにずらす
    if(leg==3)FPos.x -=0.5*stride;//0.5歩幅を後ろにずらす

    float rad = 0;
    float pi = glm::pi<float>();
    //旋回する場合の角度指定．各脚で旋回角が違う
    if(RotateMode == 1)
    {
      if(leg ==0)rad = 135 * pi / 180.0; // 回転角度をラジアンに変換
      else if(leg ==1)rad = -135 * pi / 180.0;
      else if(leg ==2)rad = 45 * pi / 180.0;
      else if(leg ==3)rad = -45 * pi / 180.0;
    }
    else if(RotateMode == -1)
    {
      if(leg ==0)rad = -45 * pi / 180.0; // 回転角度をラジアンに変換
      else if(leg ==1)rad = 45 * pi / 180.0;
      else if(leg ==2)rad = 135 * pi / 180.0;
      else if(leg ==3)rad = -135 * pi / 180.0;
    }
    else
    {      
      rad = 0;
    }

      //Z軸回転の行列を作って、遊脚ベクトルを回転させる
      glm::mat4 rot = glm::rotate(glm::mat4(1.0f), rad, glm::vec3(0.0f, 0.0f, 1.0f));
      glm::vec3 FPos_rot = glm::vec3(rot *  glm::vec4(FPos, 1.0f));


    glm::vec3 rotatedPos = FB_vec[leg]+ FPos_rot;
    SetFootPosIKBodyCoordinateToRobotState(leg, rotatedPos, state_out);

    String log_buffer = "Trot: leg=" + String(leg) + ", phase=" + String(leg_phase) + ", x=" + String(FPos.x) + ", y=" + String(FPos.y) + ", z=" + String(FPos.z) + ", Rot=" + String(RotateMode);
    Serial.println(log_buffer);
  }

}
//---------------------------------------------

#endif