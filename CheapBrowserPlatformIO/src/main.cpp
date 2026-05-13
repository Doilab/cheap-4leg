#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// --- ここからWi-Fi設定 ---
#include <WiFi.h>
#include <WebServer.h>

const char* ap_ssid = "M5Atom_Robot_test";//他の人と違うssidにする
const char* ap_pass = "12345678";
WebServer server(80);


//#define SERVO_MIN 102
//#define SERVO_MAX 512
#define SERVO_MIN 125 //2025.11.13実験 -90度(1,2軸)
#define SERVO_MAX 525 //2025.11.13実験 +90度
#define SERVO_FREQ 50

//enum LegIndex { LEG1 = 0, LEG2, LEG3, LEG4 };
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


//関節角度配列初期(Leg角)
float initialAngles[4][3] = {
  {0.0, 0.0, 0.0}, // LEG1
  {0.0, 0.0, 0.0}, // LEG2
  {0.0, 0.0, 0.0}, // LEG3
  {0.0, 0.0, 0.0}, // LEG4
};
//関節角度配列１(Leg角)
float Angles1[4][3] = {
  {0.0, 0.0, 90.0}, // LEG1
  {0.0, 0.0, 90.0}, // LEG2
  {0.0, 0.0, 90.0}, // LEG3
  {0.0, 0.0, 90.0}  // LEG4
};
//関節角度配列2(Leg角)
float Angles2[4][3] = {
  {0.0, -45.0, 90.0}, // LEG1
  {0.0, -45.0, 90.0}, // LEG2
  {0.0, -45.0, 90.0}, // LEG3
  {0.0, -45.0, 90.0}  // LEG4
};
//関節角度配列3(Leg角)
float Angles3[4][3] = {
  {20.0, 0.0, 90.0}, // LEG1
  {20.0, 0.0, 90.0}, // LEG2
  {20.0, 0.0, 90.0}, // LEG3
  {20.0, 0.0, 90.0}  // LEG4
};
//関節角度配列4(Leg角)
float Angles4[4][3] = {
  {0.0, 51.2, 25.0}, // LEG1
  {0.0, 51.2, 25.0}, // LEG2
  {0.0, 51.2, 25.0}, // LEG3
  {0.0, 51.2, 25.0}  // LEG4
};
//関節角度配列 逆運動学結果格納用(Leg角)
float AnglesIK[4][3] = {
  {0.0, 0.0, 0.0}, // LEG1
  {0.0, 0.0, 0.0}, // LEG2
  {0.0, 0.0, 0.0}, // LEG3
  {0.0, 0.0, 0.0}, // LEG4
};
//ロボット脚座標記録用
float FootPos[4][3] = {
  {0.0, 0.0, 0.0}, // LEG1xyz
  {0.0, 0.0, 0.0}, // LEG2xyz
  {0.0, 0.0, 0.0}, // LEG3xyz
  {0.0, 0.0, 0.0}, // LEG4xyz
};

////補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
//float OffsetAngles[4][3] = {
//  {-45, 10, -30}, // LEG1左前
//  {-40, 3, -0}, // LEG2左後
//  {0, 8, -14}, // LEG3 右後
//  {10, 22, -20}  // LEG4右前
//};

//補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
float OffsetAngles[4][3] = {
  {-10.0, 0.0, 2.0}, // LEG1
  {-7.0, -5.0, -5.0}, // LEG2
  {0.0, 5.0, 0.0}, // LEG3
  {-10.0, 0.0, -5.0}  // LEG4
};

Adafruit_PWMServoDriver pwm;// = Adafruit_PWMServoDriver(0x40);

//----------------------------------------------
uint16_t angleToPulse(float angle_deg) {
  //角度からサーボパルスへ変換
  //  return map((int)angle_deg, 0, 180, SERVO_MIN, SERVO_MAX);
  return map((int)angle_deg, -90, 90, SERVO_MIN, SERVO_MAX);//センターがゼロになるように直した
}

void moveServo(uint8_t channel, float angle_degS) {
  //角度指令でサーボを動かす（Servo角）
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
  for (int leg = 0; leg < 4; ++leg) {
    float th1S = Angles_array_in[leg][0]+OffsetAngles[leg][0];
    float th2S = -Angles_array_in[leg][1]+OffsetAngles[leg][1];
    float th3S = Angles_array_in[leg][2]+OffsetAngles[leg][2]-90;
    moveLeg(leg, th1S, th2S, th3S);
  }
}

char calcIK_Leg(double *th1L_out, double *th2L_out, double *th3L_out, 
double x, double y, double z) {
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


void SetPosIKLegCoordinate(int legIndex, double x, double y, double z)
{
  //各脚座標で逆運動学
  //結果は配列に格納
    double th1_tmp,th2_tmp,th3_tmp;//Leg角．degree
    char buf1[64];
    calcIK_Leg(&th1_tmp, &th2_tmp, &th3_tmp, x,y,z);
    sprintf(buf1, "LegPos%d(%.1f, %.1f,%.1f)  ->  AngL(%.1f, %.1f,%.1f)",
      legIndex, x,y,z, th1_tmp,th2_tmp,th3_tmp);
    Serial.println(buf1);
    AnglesIK[legIndex][0]=th1_tmp;
    AnglesIK[legIndex][1]=th2_tmp;
    AnglesIK[legIndex][2]=th3_tmp;
   
}

void SetPosIKBodyCoordinate(int legIndex, double x, double y, double z)
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
      sprintf(buf,"SetPosIKBodyCoordinate() (x,y,z)=(%.0f,%.0f,%.0f)",x,y,z);//debug
      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x2,y2,z)=(%.0f,%.0f,%.0f)",x2,y2,z);//debug
 //      Serial.println(buf);//debug
 //      sprintf(buf,"SetPosIKBodyCoordinate() (x3,y3,z)=(%.0f,%.0f,%.0f)",x3,y3,z);//debug
 //      Serial.println(buf);//debug
    SetPosIKLegCoordinate(legIndex,x3,y3,z);
}

//---------------------------------------------
void Bowing(void)
{
  //お辞儀をする関数
  // --- ここから「お辞儀」の修正済みコード ---
  Serial.println("Web:　slow-bowing");

  // 1. 開始姿勢（元の体勢）
  float start[4][3] = {
    {  20,  110, -80}, // 0
    {-105,  110, -80}, // 1
    { -80, -110, -80}, // 2
    {  40, -110, -80}  // 3
  };

  // 2. 目標のお辞儀姿勢
  float target[4][3] = {
    {  50,  110, -50}, // 0
    {-105,  110, -80}, // 1
    { -80, -110, -80}, // 2
    {  80, -110, -50}  // 3
  };

  int steps = 20; 

  // --- A. お辞儀をする動作（スロー） ---
  for (int s = 1; s <= steps; s++) {
    float ratio = (float)s / (float)steps;
    for (int i = 0; i < 4; i++) {
      float x = start[i][0] + (target[i][0] - start[i][0]) * ratio;
      float y = start[i][1] + (target[i][1] - start[i][1]) * ratio;
      float z = start[i][2] + (target[i][2] - start[i][2]) * ratio;
      SetPosIKBodyCoordinate(i, x, y, z);
      FootPos[i][0] = x; FootPos[i][1] = y; FootPos[i][2] = z;
    }
    SetAnglesFromArray(AnglesIK);
    delay(25); 
  }

  // --- B. 2秒間停止 ---
  Serial.println("Web: 2sec wait...");
  delay(2000); 

  // --- C. 元の体勢に戻る動作（スロー） ---
  Serial.println("Web: return");
  for (int s = 1; s <= steps; s++) {
    float ratio = (float)s / (float)steps;
    for (int i = 0; i < 4; i++) {
      // targetからstartへ向かって計算
      float x = target[i][0] + (start[i][0] - target[i][0]) * ratio;
      float y = target[i][1] + (start[i][1] - target[i][1]) * ratio;
      float z = target[i][2] + (start[i][2] - target[i][2]) * ratio;
      SetPosIKBodyCoordinate(i, x, y, z);
      FootPos[i][0] = x; FootPos[i][1] = y; FootPos[i][2] = z;
    }
    SetAnglesFromArray(AnglesIK);
    delay(25); 
  }
  Serial.println("Web: bowing done.");
  // --- ここまで ---
}

//---------------------------------------------
void ICrawl(int mode)
{
  double stride[]={40,0,0};//歩幅ベクトル
  double height = 80;//胴体高さ
  double step_height = 20;//遊脚高さ
  double dx_init = 10;//初期着地点をx方向に広げる幅
  double base_x = 50;//脚付け根x座標（Body座標）
  double base_y = 50;//脚付け根y座標（Body座標）
  double COG[]={0,0};//重心の補正
  int leg;//動かす箇所
  int swing_order[]={1,0,2,3};//遊脚順
  
    //基準姿勢
      FootPos[0][0]=base_x-1.0*stride[0]+dx_init+COG[0];
      FootPos[0][1]=base_y-1.0*stride[1]+60+COG[1];
      FootPos[0][2]=-height-1.0*stride[2];
      FootPos[1][0]=-base_x-1.0*stride[0]-dx_init+COG[0];
      FootPos[1][1]=base_y-1.0*stride[1]+60+COG[1];
      FootPos[1][2]=-height-1.0*stride[2];
      FootPos[2][0]=-base_x-0.5*stride[0]-dx_init+COG[0];
      FootPos[2][1]=-base_y-0.5*stride[1]-60+COG[1];
      FootPos[2][2]=-height-0.5*stride[2];
      FootPos[3][0]=base_x-0.5*stride[0]+dx_init+COG[0];
      FootPos[3][1]=-base_y-0.5*stride[1]-60+COG[1];
      FootPos[3][2]=-height-0.5*stride[2];

    if(mode>=1)//２遊脚(leg==1)
    {
      leg=swing_order[0];
      FootPos[leg][2]+=step_height;
    }
    if(mode>=2)//２遊脚復帰(leg==1)
    {
      for(int i=0;i<3;i++)
      FootPos[leg][i]+=stride[i];
    }
    if(mode>=3)//２遊脚下ろす(leg==1)
    {
      FootPos[leg][2]-=step_height;
    }
    
    if(mode>=4)//1遊脚(leg==0)
    {
      leg=swing_order[1];
      FootPos[leg][2]+=step_height;
    }
    if(mode>=5)//1遊脚復帰(leg==0)
    {
      for(int i=0;i<3;i++)
      FootPos[leg][i]+=stride[i];
    }
    if(mode>=6)//1遊脚下ろす(leg==0)
    {
      FootPos[leg][2]-=step_height;
    }

    if(mode>=7)//胴体推進
    {
      for(int i=0;i<4;i++)
      {
        for(int j=0;j<3;j++)
        {
          FootPos[i][j]-=stride[j]*0.5;//胴体座標系では脚先が後ろに半歩下がるように見える．
        }
      }
    }

    if(mode>=8)//3遊脚(leg==2)
    {
      leg=swing_order[2];
      FootPos[leg][2]+=step_height;
    }
    if(mode>=9)//3遊脚復帰(leg==2)
    {
      for(int i=0;i<3;i++)
      FootPos[leg][i]+=stride[i];
    }
    if(mode>=10)//3遊脚下ろす(leg==2)
    {
      FootPos[leg][2]-=step_height;
    }
    
    if(mode>=11)//4遊脚(leg==3)
    {
      leg=swing_order[3];
      FootPos[leg][2]+=step_height;
    }
    if(mode>=12)//4遊脚復帰(leg==3)
    {
      for(int i=0;i<3;i++)
      FootPos[leg][i]+=stride[i];
    }
    if(mode>=13)//4遊脚下ろす(leg==3)
    {
      FootPos[leg][2]-=step_height;
    }

    if(mode>=14)//胴体推進
    {
      for(int i=0;i<4;i++)
      {
        for(int j=0;j<3;j++)
        {
          FootPos[i][j]-=stride[j]*0.5;//胴体座標系では脚先が後ろに半歩下がるように見える．
        }
      }
    }

    //脚先座標を関節角に反映
    for(int i=0;i<4;i++)
    {
      SetPosIKBodyCoordinate(i, FootPos[i][0],FootPos[i][1],FootPos[i][2]);
    }
    SetAnglesFromArray(AnglesIK);
  
}
//----------------------------------------------
//後退歩行のモーション生成
void BCrawl(int mode) {
  double stride[] = {-40, 0, 0}; // 後ろへ
  double h = 80, sh = 20, dx = 10, bx = 50, by = 50;
  int order[] = {1, 0, 2, 3}, leg;

  // w(ICrawl 0)と同じ基準姿勢
  FootPos[0][0]=bx+dx; FootPos[0][1]=by+60; FootPos[0][2]=-h;
  FootPos[1][0]=-bx-dx; FootPos[1][1]=by+60; FootPos[1][2]=-h;
  FootPos[2][0]=-bx-dx; FootPos[2][1]=-by-60; FootPos[2][2]=-h;
  FootPos[3][0]=bx+dx; FootPos[3][1]=-by-60; FootPos[3][2]=-h;

  if (mode>=1 && mode<=3) { leg=order[0]; if(mode==1) FootPos[leg][2]+=sh; if(mode==2) FootPos[leg][0]+=stride[0]; }
  else if (mode>=4 && mode<=6) { leg=order[1]; if(mode==4) FootPos[leg][2]+=sh; if(mode==5) FootPos[leg][0]+=stride[0]; }
  else if (mode==7) { for(int i=0;i<4;i++) FootPos[i][0]-=stride[0]*0.5; }
  else if (mode>=8 && mode<=10) { leg=order[2]; if(mode==8) FootPos[leg][2]+=sh; if(mode==9) FootPos[leg][0]+=stride[0]; }
  else if (mode>=11 && mode<=13) { leg=order[3]; if(mode==11) FootPos[leg][2]+=sh; if(mode==12) FootPos[leg][0]+=stride[0]; }
  else if (mode==14) { for(int i=0;i<4;i++) FootPos[i][0]-=stride[0]*0.5; }

  for (int i=0; i<4; i++) SetPosIKBodyCoordinate(i, FootPos[i][0], FootPos[i][1], FootPos[i][2]);
  SetAnglesFromArray(AnglesIK);
}
//---------------------------------------------
//以下テスト関数群
//---------------------------------------------
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
      int ch =7;
      sprintf(buf,"PWM ch[%d] value[%d]",ch,value);
      Serial.println(buf);
      pwm.setPWM(ch, 0, value);//生のPWM設定．モータ１個テスト用
    }
 
  }
}
//---------------------------------------------
void moveServo_test(void)
{
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
      int ch =7;
      sprintf(buf,"moveServo ch[%d] value[%d]",ch,value);
      Serial.println(buf);
      moveServo(ch, value);//角度入力（サーボ角）．モータ１個テスト用
    }
 
  }
}
//---------------------------------------------
void SetAnglesFromArray_test(void)
{
  char buf[64];
  Serial.println("SetAnglesFromArray_test()1,2,3,4 or q");
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
    else if(str1=="1")
    {
      sprintf(buf,"SetAnglesFromArray 1");
      Serial.println(buf);
      SetAnglesFromArray(Angles1);
    }
    else if(str1=="2")
    {
      sprintf(buf,"SetAnglesFromArray 2");
      Serial.println(buf);
      SetAnglesFromArray(Angles2);
    }
    else if(str1=="3")
    {
      sprintf(buf,"SetAnglesFromArray 3");
      Serial.println(buf);
      SetAnglesFromArray(Angles3);
    }
    else if(str1=="4")
    {
      sprintf(buf,"SetAnglesFromArray 4");
      Serial.println(buf);
      SetAnglesFromArray(Angles4);
    }
 
  }
}

//---------------------------------------------
void SetPosIKLegCoordinate_test(void)
{
  //脚座標で逆運動学のテスト
  char buf[64];
  Serial.println("SetPosIKLegCoordinate_test()1 or 2 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("SetPosIKLegCoordinate_test()--quit");
      return;
    }
    if(str1=="1")
    {
      float x=90;
      float y=0;
      float z=-80;
      
      sprintf(buf,"SetPosIKLegCoordinate (%.1f, %.1f, %.1f )",x,y,z);
      Serial.println(buf);
      SetPosIKLegCoordinate(0,x,y,z);
      SetPosIKLegCoordinate(1,x,y,z);
      SetPosIKLegCoordinate(2,x,y,z);
      SetPosIKLegCoordinate(3,x,y,z);
      SetAnglesFromArray(AnglesIK);
    }
    else if(str1=="2")
    {
      float x=90;
      float y=10;
      float z=-70;
      
      sprintf(buf,"SetPosIKLegCoordinate (%.1f, %.1f, %.1f )",x,y,z);
      Serial.println(buf);
      SetPosIKLegCoordinate(0,x,y,z);
      SetPosIKLegCoordinate(1,x,y,z);
      SetPosIKLegCoordinate(2,x,y,z);
      SetPosIKLegCoordinate(3,x,y,z);
      SetAnglesFromArray(AnglesIK);
    }
 
  }
}
//---------------------------------------------
void SetPosIKBodyCoordinate_test(void)
{
  //胴体座標で逆運動学のテスト
  char buf[64];
  Serial.println("SetPosIKBodyCoordinate_test() 1 or 2 or q");
  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    str1.trim();
    Serial.println(str1);
    if(str1=="q")
    {
      Serial.println("SetPosIKBodyCoordinate_test()--quit");
      return;
    }
    if(str1=="1")
    {
      double x=140;
      double y=50;
      double z=-80;
      sprintf(buf,"Body coordinate (%.0f, %.0f, %.0f)",x,y,z);
      Serial.println(buf);
      SetPosIKBodyCoordinate(0,x,y,z);
      SetPosIKBodyCoordinate(1,-x,y,z);
      SetPosIKBodyCoordinate(2,-x,-y,z);
      SetPosIKBodyCoordinate(3,x,-y,z);
      SetAnglesFromArray(AnglesIK);
    }
    if(str1=="2")
    {
      SetPosIKBodyCoordinate(0,50,140,-50);
      SetPosIKBodyCoordinate(1,-50,140,-50);
      SetPosIKBodyCoordinate(2,-50,-140,-50);
      SetPosIKBodyCoordinate(3,50,-140,-50);
      SetAnglesFromArray(AnglesIK);
    }
 
  }
}
//---------------------------------------------
void WalkTest(int repetitions) //引数を繰り返し回数(repetitions)に変更
{
  Serial.println("Walk Start");
  ICrawl(0);
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (int i = 0; i < 16; i++) {
      ICrawl(i);
      delay(300); // 歩行速度（ミリ秒）
    }
  }

  ICrawl(0);
  Serial.println("Walk End");
}

void BackTest(int repetitions) {
  Serial.println("Back Walk Start");
  BCrawl(0); // 後退の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (int i = 0; i < 16; i++) {
      BCrawl(i); // 後退用の16ステップを動かす
      delay(300); 
    }
  }

  BCrawl(0); // 最後に止まる
  Serial.println("Back Walk End");
}

//---------------------
// スマホに表示される操作画面
void handleRoot() {
  String html = "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
  html += "<style>";
  // 背景設定
  html += "body { background-color: #2d2d2d; color: white; font-family: sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; margin: 0; overflow: hidden; }";
  html += "h1 { font-size: 28px; margin-bottom: 25px; letter-spacing: 2px; }";
  
  // グリッド配置
  html += ".grid { display: grid; grid-template-columns: repeat(2, 140px); grid-gap: 30px; }";
  
  // 基本ボタン設定（太い黒枠の白い正方形）
  html += "button { width: 140px; height: 140px; background-color: white; border: 8px solid #000; position: relative; font-size: 24px; font-weight: bold; cursor: pointer; transition: 0.1s; display: flex; align-items: center; justify-content: center; z-index: 1; }";
  html += "button:active { transform: scale(0.95); opacity: 0.9; }";

  // --- 枠で囲う設定（beforeとafterを使って色枠をずらして配置） ---
  // 共通設定：ボタンの背後に色付きの枠を作る
  html += "button::before { content:''; position:absolute; top:-12px; left:-12px; right:-12px; bottom:-12px; border:3px solid currentColor; z-index: -1; pointer-events:none; }";

  // 各ボタンの色指定
  html += ".c-red { color: #ff4d4d; }";    // 前進（赤）
  html += ".c-blue { color: #00a8ff; }";   // 後退（青）
  html += ".c-yellow { color: #ffbc00; }"; // お辞儀（黄）
  html += ".c-green { color: #2ed573; }";  // リセット（緑）

  html += ".footer { margin-top: 35px; font-size: 12px; color: #777; }";
  html += "</style></head><body>";

  html += "<h1>ロボット制御 Cheap４脚</h1>";
  
  html += "<div class='grid'>";
  // 各ボタン（classで色を呼び出し）
  html += "  <button class='c-red' onclick=\"fetch('/w')\">前進</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/b')\">後退</button>";
  html += "  <button class='c-yellow' onclick=\"fetch('/h')\">お辞儀</button>";
  html += "  <button class='c-green' onclick=\"fetch('/reset')\">リセット</button>";
  html += "</div>";

  html += "<div class='footer'>M5Atom S3 Controller</div>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Wi-Fi専用の初期設定
void setupWiFi() {
  Serial.println("--- Wi-Fi Setup Start ---");

  // Wi-Fiの親機モードを開始
  if (WiFi.softAP(ap_ssid, ap_pass)) {
    Serial.println("Wi-Fi AP Started !!");
    Serial.print("SSID: "); Serial.println(ap_ssid);
    Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Wi-Fi AP Failed...");
  }

  // スマホ操作用サーバーのボタン処理設定
  server.on("/", handleRoot);//再表示
  server.on("/reset", []() { ICrawl(0); server.send(200, "text/plain", "OK"); });//間歇クロールの初期状態へ
  server.on("/w", []() { WalkTest(1); server.send(200, "text/plain", "OK"); });//歩行開始関数を呼び出す
  server.on("/b", []() { BackTest(1); server.send(200, "text/plain", "OK"); });//後退関数を呼び出す
  server.on("/h", []() { Bowing(); server.send(200, "text/plain", "OK"); });//お辞儀


  server.begin();
  Serial.println("HTTP Server Started");
  Serial.println("--- Wi-Fi Setup Done ---");
}
//------------------------------------------------------------
//---------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("--- Booting Robot ---");
  delay(1000); // 起動直後に少し待機
  
  
  // 先にWi-Fiを立ち上げる
  Serial.println("--- Initializing Wi-Fi ---");
  setupWiFi(); 

  // その後にサーボなどの設定をする
  Serial.println("--- Initializing Servos ---");
  Wire.begin(38,39); // I2Cのピンを指定（SDA=38, SCL=39）
  pwm = Adafruit_PWMServoDriver(0x40);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  Serial.println("System Ready.");
  ICrawl(0); // 基準姿勢へ
}

//---------------------------------------------
void loop() {
  // 常にスマホからのアクセスをチェック（止めてはいけない）
  server.handleClient();

  // パソコンからの入力があるときだけ、以下の処理を行う
  if (Serial.available() > 0) {
    String str1 = Serial.readString();
    str1.trim();
    Serial.println("Received: " + str1);

    if (str1 == "1") {
      PWM_test();
    } else if (str1 == "2") {
      moveServo_test();
    } else if (str1 == "3") {
      SetAnglesFromArray_test();
    } else if (str1 == "4") {
      SetPosIKLegCoordinate_test();
    } else if (str1 == "5") {
      SetPosIKBodyCoordinate_test();
    } else if (str1.startsWith("w")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        WalkTest(count);
      } else {
        Serial.println("Reset Forward Pose");
        ICrawl(0);
      }
    } else if (str1.startsWith("b")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        BackTest(count);
      } else {
        Serial.println("Reset Backward Pose");
        BCrawl(0);
      }
    } else if (str1 == "h") {
      Serial.println("Hello!");
      SetPosIKBodyCoordinate(0, 140, 50, -40); SetPosIKBodyCoordinate(1, -140, 50, -40);
      SetPosIKBodyCoordinate(2, -140, -50, -80); SetPosIKBodyCoordinate(3, 140, -50, -80);
      SetAnglesFromArray(AnglesIK);
      delay(1000);
      ICrawl(0);
    } else {
      SetAnglesFromArray(initialAngles);
    }
    
    // 次の命令を促す表示（シリアル入力があった時だけ出す）
    Serial.println("1:PWM, 2:moveservo, 3:array, 4:IKLeg, 5:IKBody, w:walk, b:back, h:hello");
  }

  // ループが速すぎると通信が不安定になることがあるため、ごくわずかに待機
  delay(10);
}
//---------------------------------------------