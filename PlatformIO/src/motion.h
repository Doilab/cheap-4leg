//モーションを記述する
#ifndef MOTION_H
#define MOTION_H
#include <math.h>
 //---------------------------------------------

//---------------------------------------------
void SetInitialPose(float Angles_array_out[4][3])
{
  //初期姿勢にする関数
  //関節角度配列初期(Leg角)
  float initialAngles[4][3] = {
    {0.0, 0.0, 0.0}, // LEG1
    {0.0, 0.0, 0.0}, // LEG2
    {0.0, 0.0, 0.0}, // LEG3
    {0.0, 0.0, 0.0}, // LEG4
  };
  for (int leg = 0; leg < 4; ++leg) {
    SetFootPosIKLegCoordinateToArray(leg, initialAngles[leg][0], initialAngles[leg][1], initialAngles[leg][2], Angles_array_out);
  }
}
//---------------------------------------------
void RectangleFootMotion(double phase, double *x_out, double *y_out, double *z_out)
{
  //脚先  １個分を長方形状に動かす動作生成の関数
  double stride = 40;//歩幅
  double f_height = 20;//遊脚高さ
  double x,y,z;
  double dp, ratio;
  
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

  *x_out=x;
  *y_out=y;
  *z_out=z;
  return;
}
//---------------------------------------------
void Trot(double phase, int RotateMode, float Angles_array_out[4][3])
{
  //トロット歩容のモーション生成の関数
  double height = 80;//胴体高さ
  double dx_init = 10;//初期着地点をx方向に広げる幅
  double dy_init = 60;//初期着地点をy方向に広げる幅
  double base_x = 50;//脚付け根x座標（Body座標）
  double base_y = 50;//脚付け根y座標（Body座標）
  double COG[]={0,0};//重心の水平方向補正
  int leg;//動かす箇所

  double FootPos[4][3];//脚先位置の配列

  //歩容軌道の中心点
  double FB[4][3];
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

  double x,y,z;
  char log_buffer[100];
  for(leg=0;leg<4;leg++)
  {
    double leg_phase = 0;
    // 各脚の位相をずらす．
    if(leg == 0) leg_phase = fmod(phase + 0.00, 1.0);//
    if(leg == 2) leg_phase = fmod(phase + 0.00, 1.0);//
    if(leg == 1) leg_phase = fmod(phase + 0.50, 1.0);// 
    if(leg == 3) leg_phase = fmod(phase + 0.50, 1.0);//
    //脚の動きの生成
    RectangleFootMotion(leg_phase, &x, &y, &z);

    double rad = 0;
    if(RotateMode == 1)
    {
      if(leg ==0)rad = 135 * M_PI / 180.0; // 回転角度をラジアンに変換
      else if(leg ==1)rad = -135 * M_PI / 180.0;
      else if(leg ==2)rad = 45 * M_PI / 180.0;
      else if(leg ==3)rad = -45 * M_PI / 180.0;
    }
    else if(RotateMode == -1)
    {
      if(leg ==0)rad = -45 * M_PI / 180.0; // 回転角度をラジアンに変換
      else if(leg ==1)rad = 45 * M_PI / 180.0;
      else if(leg ==2)rad = 135 * M_PI / 180.0;
      else if(leg ==3)rad = -135 * M_PI / 180.0;
    }
    else
    {      
      rad = 0;
    }
      double cos_rad = cos(rad);
      double sin_rad = sin(rad);
    
      // 胴体回転の適用
      double x_rot = x * cos_rad - y * sin_rad;
      double y_rot = x * sin_rad + y * cos_rad;
      x = x_rot;
      y = y_rot;
    
    
    FootPos[leg][0]=FB[leg][0]+x;
    FootPos[leg][1]=FB[leg][1]+y;
    FootPos[leg][2]=FB[leg][2]+z;
    SetFootPosIKBodyCoordinateToArray(leg, FootPos[leg][0], FootPos[leg][1], FootPos[leg][2], Angles_array_out);

    sprintf(log_buffer, "Trot: leg=%d, phase=%.2f, x=%.1f, y=%.1f, z=%.1f Rot=%d", leg, leg_phase, x, y, z, RotateMode);
    Serial.println(log_buffer);
  }

}
//---------------------------------------------
void ICRectangleFootMotion(double phase, double *x_out, double *y_out, double *z_out, char flagFrontLeg )
{
  //脚先  １個分を長方形状に動かす動作生成の関数
  //間歇クロール専用
  double stride = 40;//歩幅
  double f_height = 20;//遊脚高さ
  double x,y,z;
  double dp, ratio;
  
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
 
  *x_out=x;
  *y_out=y;
  *z_out=z;
  return;
}
//---------------------------------------------
void ICrawl(double phase, float Angles_array_out[4][3])
{
  //間歇クロール歩容のモーション生成の関数
  double height = 80;//胴体高さ
  double dx_init = 10;//初期着地点をx方向に広げる幅
  double dy_init = 60;//初期着地点をy方向に広げる幅
  double base_x = 50;//脚付け根x座標（Body座標）
  double base_y = 50;//脚付け根y座標（Body座標）
  double COG[]={0,0};//重心の水平方向補正
  int leg;//動かす箇所

  double FootPos[4][3];//脚先位置の配列

  //歩容軌道の中心点
  double FB[4][3];
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

  double x,y,z;
  char log_buffer[100];
  for(leg=0;leg<4;leg++)
  {
    double leg_phase = 0;
    // 左右半身で各脚の位相をずらす．
    if(leg == 1) leg_phase = fmod(phase + 0.00, 1.0);//左半身 
    if(leg == 0) leg_phase = fmod(phase + 0.00, 1.0);//左半身
    if(leg == 2) leg_phase = fmod(phase + 0.50, 1.0);//右半身
    if(leg == 3) leg_phase = fmod(phase + 0.50, 1.0);//右半身
    //脚の動きの生成
    if((leg == 1)||(leg==2))ICRectangleFootMotion(leg_phase, &x, &y, &z, 0);
    if((leg == 0)||(leg==3))ICRectangleFootMotion(leg_phase, &x, &y, &z, 1);

    FootPos[leg][0]=FB[leg][0]+x;
    FootPos[leg][1]=FB[leg][1]+y;
    FootPos[leg][2]=FB[leg][2]+z;
    SetFootPosIKBodyCoordinateToArray(leg, FootPos[leg][0], FootPos[leg][1], FootPos[leg][2], Angles_array_out);

    sprintf(log_buffer, "ICrawl2: leg=%d, phase=%.2f, x=%.1f, y=%.1f, z=%.1f", leg, leg_phase, x, y, z);
    Serial.println(log_buffer);
  }
}
//---------------------------------------------
//後退歩行のモーション生成
void ICrawl_Back(double phase, float Angles_array_out[4][3])
 {
  double r_phase = fmod(1-phase,1.0);//ICrawlのモードを逆順にする
  ICrawl(r_phase, Angles_array_out);
} 
#endif