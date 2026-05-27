//運動学の処理

#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>//piのため
#include <glm/vec3.hpp>//vec3のため
#include <cmath>//数学関数．atan2, sqrt, acosなど．

// Cheap４脚のリンク長 [mm]
const float LINK1_LENGTH = 30.0;
const float LINK2_LENGTH = 60.0;
const float LINK3_LENGTH = 80.0;


//関節角度定義(Servo角)
//th1S 反時計回り正　付け根のでっぱりに沿った斜め方向ゼロの基準
//th2S　上がる向きが正．水平基準
//th3S　たたむ向きが正．第２リンクに対して直角下向きが基準
//関節角度定義(Leg角)
//th1 反時計回り正　付け根のでっぱりに沿った斜め方向ゼロの基準
//th2　下がる向きが正．水平基準
//th3　たたむ向きが正．第２リンクと一直線上が基準

//脚定義
//LEG1 左前
//LEG2 左後
//LEG3 右後
//LEG4 右前


//----------------------------------------------
//ロボットの３自由度脚状態を保持する構造体
struct LegState
{
    glm::vec3 footPos;//Leg座標での足先位置
    glm::vec3 jointAngles;//Leg角での関節角度
};

//ロボットの状態を保持する構造体
struct RobotState
{
    LegState legs[4];
};

//----------------------------------------------

//---------------------------------------------
char calcIK_Leg(float x, float y, float z,
         float *th1L_out, float *th2L_out, float *th3L_out) 
{
  //各脚座標系で逆運動学（Leg角.degree）
  //途中計算はradian
  //エラーチェックが必要だがとりあえず動く範囲で動かす前提で作る．
  float l1 = LINK1_LENGTH;
  float l2 = LINK2_LENGTH;
  float l3 = LINK3_LENGTH;

  float pi = glm::pi<float>();
  float th11 = std::atan2(y,x);
  float r = std::sqrt(x*x+y*y);
  float s = std::sqrt(pow((r-l1),2)+z*z);
  float abc = std::acos((l2*l2+l3*l3-s*s)/(2*l2*l3));
  float th13 = pi-abc;
  float alpha = std::atan2(-z,(r-l1));
  float bac = std::acos((l2*l2+s*s-l3*l3)/(2*l2*s));
  float th12 = alpha - bac;

  *th1L_out=th11*180.0/pi;
  *th2L_out=th12*180.0/pi;
  *th3L_out=th13*180.0/pi;
  return 1;

  //return -1;//エラーの場合
}

void SetFootPosIKLegCoordinateToArray(int legIndex, float x, float y, float z, float Angles_array_out[4][3])
{
  //各脚座標で逆運動学
  //結果は配列に格納
    float th1_tmp,th2_tmp,th3_tmp;//Leg角．degree
    char buf1[64];
    calcIK_Leg(x,y,z, &th1_tmp, &th2_tmp, &th3_tmp);
    // sprintf(buf1, "LegPos%d(%.1f, %.1f,%.1f)  ->  AngL(%.1f, %.1f,%.1f)",
    //   legIndex, x,y,z, th1_tmp,th2_tmp,th3_tmp);
    //Serial.println(buf1);
    Angles_array_out[legIndex][0]=th1_tmp;
    Angles_array_out[legIndex][1]=th2_tmp;
    Angles_array_out[legIndex][2]=th3_tmp;
   
}

glm::vec3 SetFootPosIKLegCoordinateToVec3(glm::vec3 pos)
{
  //各脚座標で逆運動学
  //結果はvec3で出力
    float th1_tmp,th2_tmp,th3_tmp;//Leg角．degree
    char buf1[64];
    char flag = calcIK_Leg(pos.x,pos.y,pos.z, &th1_tmp, &th2_tmp, &th3_tmp);
    // sprintf(buf1, "LegPos%d(%.1f, %.1f,%.1f)  ->  AngL(%.1f, %.1f,%.1f)",
    //   legIndex, pos.x,pos.y,pos.z, th1_tmp,th2_tmp,th3_tmp);
    //Serial.println(buf1);

    glm::vec3 angles_out(th1_tmp, th2_tmp, th3_tmp);
    return angles_out;
    
}


char SetFootPosIKBodyCoordinateToRobotState(int legIndex, glm::vec3 pos, RobotState *state_out)
{
  char flag = 1;
  //胴体座標で逆運動学
  float x2,y2,x3,y3,z;
  float th;
  float offset_x, offset_y;
  char buf[64];
  float pi = glm::pi<float>();
 
  if(legIndex==0)
  {
    th=pi/4;
    offset_x=50;//胴体中心から見た回転軸のオフセット
    offset_y=50;//胴体中心から見た回転軸のオフセット
  }
  else if(legIndex==1)
  {
    th=pi*3/4;
    offset_x=-50;
    offset_y=50;
  }
  else if(legIndex==2)
  {
    th=-pi*3/4;
    offset_x=-50;
    offset_y=-50;
  }
  else if(legIndex==3)
  {
    th=-pi*1/4;
    offset_x=50;
    offset_y=-50;
  }
  else
  {
    flag = -1;
    return flag;
  }
      x2=pos.x-offset_x;
      y2=pos.y-offset_y;
      x3=x2*cos(-th)-y2*sin(-th);
      y3=x2*sin(-th)+y2*cos(-th);
      z=pos.z;
//      sprintf(buf,"SetPosIKBodyCoordinate() (x,y,z)=(%.0f,%.0f,%.0f)",x,y,z);//debug
//      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x2,y2,z)=(%.0f,%.0f,%.0f)",x2,y2,z);//debug
 //      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x3,y3,z)=(%.0f,%.0f,%.0f)",x3,y3,z);//debug
 //      Serial.println(buf);//debug
 
 state_out->legs[legIndex].jointAngles = SetFootPosIKLegCoordinateToVec3(glm::vec3(x3,y3,z));
 state_out->legs[legIndex].footPos = pos;//脚先位置も更新しておく． 
  return flag;
}

#endif