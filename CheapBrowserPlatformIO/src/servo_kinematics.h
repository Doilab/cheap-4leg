//サーボと運動学の処理

#ifndef SERVO_KINEMATICS_H
#define SERVO_KINEMATICS_H

#include <Adafruit_PWMServoDriver.h>
#include <math.h>

//#define SERVO_MIN 102
//#define SERVO_MAX 512
#define SERVO_MIN 125 //2025.11.13実験 -90度(1,2軸)
#define SERVO_MAX 525 //2025.11.13実験 +90度
#define SERVO_FREQ 50

const uint8_t servoChannels[4][3] = {
  {0, 1, 2},    // LEG1: THETA11, 12, 13
  {3, 4, 5},    // LEG2
  {6, 7, 8},    // LEG3
  {9,10,11}     // LEG4
};

// リンク長 [mm]
const float l1 = 30.0;
const float l2 = 60.0;
const float l3 = 80.0;

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

////補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
//float OffsetAngles[4][3] = {
//  {-45, 10, -30}, // LEG1左前
//  {-40, 3, -0}, // LEG2左後
//  {0, 8, -14}, // LEG3 右後
//  {10, 22, -20}  // LEG4右前
//};

//補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
//ロボットの機体により個体差あり
float OffsetAngles[4][3] = {
  {-10.0, 0.0, 2.0}, // LEG1
  {-7.0, -5.0, -5.0}, // LEG2
  {0.0, 5.0, 0.0}, // LEG3
  {-10.0, 0.0, -5.0}  // LEG4
};

Adafruit_PWMServoDriver pwm;
bool servos_initialized=false;//サーボが初期化されているかどうかのフラグ

//----------------------------------------------
uint16_t angleToPulse(float angle_deg) {
  //Servo角からサーボパルスへ変換
  //  return map((int)angle_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  return map((int)angle_deg, -90, 90, SERVO_MIN, SERVO_MAX);//センターがゼロになるように直した
}

void init_servos() {
  //サーボの初期化
  Wire.begin(38,39); // I2Cのピンを指定（SDA=38, SCL=39）
  pwm = Adafruit_PWMServoDriver(0x40);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  servos_initialized=true;
}

void moveServo(uint8_t channel, float angle_degS) {
  //角度指令でサーボを動かす（Servo角）
  if(!servos_initialized) {
    return;//サーボが初期化されていない場合は動かさない
  } 
  angle_degS = constrain(angle_degS, -110, 110);
  uint16_t pulse = angleToPulse(angle_degS);
  pwm.setPWM(channel, 0, pulse);
}


void moveLeg(uint8_t legIndex, float th1S, float th2S, float th3S) {
  //Servo角指令で脚関節を動かす．配列でチャンネル指定
  moveServo(servoChannels[legIndex][0], th1S);
  moveServo(servoChannels[legIndex][1], th2S);
  moveServo(servoChannels[legIndex][2], th3S);
}

void SetAnglesFromArray(float Angles_array_in[4][3])
{
  //関節角度配列（Leg角）に補正配列を加えてServo角で出力
  //全関節
  for (int leg = 0; leg < 4; ++leg) {
    float th1S = Angles_array_in[leg][0]+OffsetAngles[leg][0];
    float th2S = -Angles_array_in[leg][1]+OffsetAngles[leg][1];
    float th3S = Angles_array_in[leg][2]+OffsetAngles[leg][2]-90;
    moveLeg(leg, th1S, th2S, th3S);
  }
}
//---------------------------------------------
char calcIK_Leg(double x, double y, double z,
         double *th1L_out, double *th2L_out, double *th3L_out) 
{
  //各脚座標系で逆運動学（Leg角.degree）
  //途中計算はradian
  double th11 = atan2(y,x);
  double r = sqrt(x*x+y*y);
  double s = sqrt(pow((r-l1),2)+z*z);
  double abc = acos((l2*l2+l3*l3-s*s)/(2*l2*l3));
  double th13 = M_PI-abc;
  double alpha = atan2(-z,(r-l1));
  double bac = acos((l2*l2+s*s-l3*l3)/(2*l2*s));
  double th12 = alpha - bac;

  *th1L_out=th11*180.0/M_PI;
  *th2L_out=th12*180.0/M_PI;
  *th3L_out=th13*180.0/M_PI;
  return 1;
}

void SetFootPosIKLegCoordinateToArray(int legIndex, double x, double y, double z, float Angles_array_out[4][3])
{
  //各脚座標で逆運動学
  //結果は配列に格納
    double th1_tmp,th2_tmp,th3_tmp;//Leg角．degree
    char buf1[64];
    calcIK_Leg(x,y,z, &th1_tmp, &th2_tmp, &th3_tmp);
    // sprintf(buf1, "LegPos%d(%.1f, %.1f,%.1f)  ->  AngL(%.1f, %.1f,%.1f)",
    //   legIndex, x,y,z, th1_tmp,th2_tmp,th3_tmp);
    //Serial.println(buf1);
    Angles_array_out[legIndex][0]=th1_tmp;
    Angles_array_out[legIndex][1]=th2_tmp;
    Angles_array_out[legIndex][2]=th3_tmp;
   
}

void SetFootPosIKBodyCoordinateToArray(int legIndex, double x, double y, double z, float Angles_array_out[4][3])
{
  //胴体座標で逆運動学
  double x2,y2,x3,y3;
  double th;
  double offset_x, offset_y;
  char buf[64];
  if(legIndex==0)
  {
    th=M_PI/4;
    offset_x=50;//胴体中心から見た回転軸のオフセット
    offset_y=50;//胴体中心から見た回転軸のオフセット
  }
  else if(legIndex==1)
  {
    th=M_PI*3/4;
    offset_x=-50;
    offset_y=50;
  }
  else if(legIndex==2)
  {
    th=-M_PI*3/4;
    offset_x=-50;
    offset_y=-50;
  }
  else if(legIndex==3)
  {
    th=-M_PI*1/4;
    offset_x=50;
    offset_y=-50;
  }
  else
  {
    return;
  }
      x2=x-offset_x;
      y2=y-offset_y;
      x3=x2*cos(-th)-y2*sin(-th);
      y3=x2*sin(-th)+y2*cos(-th);
//      sprintf(buf,"SetPosIKBodyCoordinate() (x,y,z)=(%.0f,%.0f,%.0f)",x,y,z);//debug
//      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x2,y2,z)=(%.0f,%.0f,%.0f)",x2,y2,z);//debug
 //      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x3,y3,z)=(%.0f,%.0f,%.0f)",x3,y3,z);//debug
 //      Serial.println(buf);//debug
    SetFootPosIKLegCoordinateToArray(legIndex,x3,y3,z, Angles_array_out);
}

#endif