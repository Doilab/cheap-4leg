//サーボと運動学の処理

#ifndef SERVO_H
#define SERVO_H

//#include <glm/vec3.hpp>
#include <Adafruit_PWMServoDriver.h>
//#include <math.h>
//#include <cmath>

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

//----------------------------------------------
Adafruit_PWMServoDriver pwm;//サーボドライバのインスタンス
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
/*void SetAnglesFromState(RobotState state)
{
  //RobotStateから関節角度を取り出してServo角で出力
  //全関節
  for (int leg = 0; leg < 4; ++leg) {
    float th1S = state.legs[leg].jointAngles[0]+OffsetAngles[leg][0];
    float th2S = -state.legs[leg].jointAngles[1]+OffsetAngles[leg][1];
    float th3S = state.legs[leg].jointAngles[2]+OffsetAngles[leg][2]-90;
    moveLeg(leg, th1S, th2S, th3S);
  }
}*/


#endif