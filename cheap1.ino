#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


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

//補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
float OffsetAngles[4][3] = {
  {-10.0, 0.0, 2.0}, // LEG1
  {-7.0, -5.0, -5.0}, // LEG2
  {0.0, 5.0, 0.0}, // LEG3
  {-10.0, 0.0, -5.0}  // LEG4
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);



uint16_t angleToPulse(float angle_deg) {
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
  //各脚座標系で逆運動学（Leg角）
  double th11 = atan2(y,x);
  double r = sqrt(x*x+y*y);
  double s = sqrt(pow((r-l1),2)+z*z);
  double abc = acos((l2*l2+l3*l3-s*s)/(2*l2*l3));
  double th13 = M_PI-abc;
  double alpha = atan2(-z,(r-l1));
  double bac = acos((l2*l2+s*s-l3*l3)/(2*l2*s));
  double th12 = alpha - bac;

  *th1L_out=th11;
  *th2L_out=th12;
  *th3L_out=th13;
  return 1;
}


void SetPosIKLegCoordinate(int legIndex, double x, double y, double z)
{
  //各脚座標で逆運動学
  //結果は配列に格納
    double th1_tmp,th2_tmp,th3_tmp;
    char buf1[64];
    calcIK_Leg(&th1_tmp, &th2_tmp, &th3_tmp, x,y,z);
    sprintf(buf1, "LegPos%d(%.0f, %.0f,%.0f)  ->  AngL(%.1f, %.1f,%.1f)",
      legIndex, x,y,z, th1_tmp*180/M_PI,th2_tmp*180/M_PI,th3_tmp*180/M_PI);
    Serial.println(buf1);
    AnglesIK[legIndex][0]=th1_tmp*180/M_PI;
    AnglesIK[legIndex][1]=th2_tmp*180/M_PI;
    AnglesIK[legIndex][2]=th3_tmp*180/M_PI;
   
}

void SetPosIKBodyCoordinate(int legIndex, double x, double y, double z)
{
  //胴体座標で逆運動学
  double x2,y2,x3,y3;
  double th;
  double offset_x, offset_y;
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
    SetPosIKLegCoordinate(legIndex,x3,y3,z);
}

//---------------------------------------------
void StandardStat(int mode)
{
  //基準姿勢を作る
  double stride = 40;//歩幅
  double height = 80;//胴体高さ
  double step_height = 20;//遊脚高さ
  double dx_init = 10;//初期着地点をx方向に広げる幅
  double base_x = 50;//脚付け根x座標（Body座標）
  double base_y = 50;//脚付け根y座標（Body座標）
  int leg;//動かす箇所
  
    //基準姿勢
      FootPos[0][0]=base_x-1.0*stride+dx_init;
      FootPos[0][1]=base_y+60;
      FootPos[0][2]=-height;
      FootPos[1][0]=-base_x-1.0*stride-dx_init;
      FootPos[1][1]=base_y+60;
      FootPos[1][2]=-height;
      FootPos[2][0]=-base_x-0.5*stride-dx_init;
      FootPos[2][1]=-base_y-60;
      FootPos[2][2]=-height;
      FootPos[3][0]=base_x-0.5*stride+dx_init;
      FootPos[3][1]=-base_y-60;
      FootPos[3][2]=-height;
      for(int i=0;i<4;i++)
      {
        SetPosIKBodyCoordinate(i, FootPos[i][0],FootPos[i][1],FootPos[i][2]);
      }

    if(mode>=1)//２遊脚(leg==1)
    {
      leg=1;
      FootPos[leg][2]+=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=2)//２遊脚復帰(leg==1)
    {
      FootPos[leg][0]+=stride;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=3)//２遊脚下ろす(leg==1)
    {
      FootPos[leg][2]-=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    
    if(mode>=4)//1遊脚(leg==0)
    {
      leg=0;
      FootPos[leg][2]+=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=5)//1遊脚復帰(leg==0)
    {
      FootPos[leg][0]+=stride;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=6)//1遊脚下ろす(leg==0)
    {
      FootPos[leg][2]-=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }

    if(mode>=7)//胴体推進
    {
      for(int i=0;i<4;i++)
      {
        FootPos[i][0]-=stride*0.5;//胴体座標系では脚先が後ろに下がるように見える．
        SetPosIKBodyCoordinate(i, FootPos[i][0],FootPos[i][1],FootPos[i][2]);
      }
    }

    if(mode>=8)//3遊脚(leg==2)
    {
      leg=2;
      FootPos[leg][2]+=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=9)//3遊脚復帰(leg==2)
    {
      FootPos[leg][0]+=stride;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=10)//3遊脚下ろす(leg==2)
    {
      FootPos[leg][2]-=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    
    if(mode>=11)//4遊脚(leg==3)
    {
      leg=3;
      FootPos[leg][2]+=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=12)//4遊脚復帰(leg==3)
    {
      FootPos[leg][0]+=stride;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }
    if(mode>=13)//4遊脚下ろす(leg==3)
    {
      FootPos[leg][2]-=step_height;
      SetPosIKBodyCoordinate(leg, FootPos[leg][0],FootPos[leg][1],FootPos[leg][2]);
    }

    if(mode>=14)//胴体推進
    {
      for(int i=0;i<4;i++)
      {
        FootPos[i][0]-=stride*0.5;//胴体座標系では脚先が後ろに下がるように見える．
        SetPosIKBodyCoordinate(i, FootPos[i][0],FootPos[i][1],FootPos[i][2]);
      }
    }

    SetAnglesFromArray(AnglesIK);
  
}
//---------------------------------------------
void WalkTest(void)
{
  //歩行モーションを作る

  //基準姿勢
  StandardStat(0);
  
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    String str1=Serial.readString();
    Serial.println(str1);

  //歩行開始
  for(int i=0;i<16;i++)
  {
    StandardStat(i);
    delay(500);
  }

  
  


  //基準にもどる
  StandardStat(0);
}

//---------------------------------------------
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(1000);
  Serial.println("Cheap-4leg Robot test program");
  Serial.println("Initial angles");
  SetAnglesFromArray(initialAngles);
  delay(1000);
  Serial.println("----");
}

//---------------------------------------------
void loop() {
  String str1;

  while(1)
  {
    while(Serial.available()<=0)//シリアルモニタで何か文字が入力されるまで待つ
    {; }
    str1=Serial.readString();
    Serial.println(str1);
    str1.trim();
    
    if(str1=="1")
      {
      //pwm.setPWM(7, 0, value);//生のPWM設定．モータ１個テスト用
      //moveServo(7, value);//角度入力．モータ１個テスト用
//      SetPosIKLegCoordinate(0,90,0,-80);
//      SetPosIKLegCoordinate(1,90,0,-80);
//      SetPosIKLegCoordinate(2,90,0,-80);
//      SetPosIKLegCoordinate(3,90,0,-80);
      SetPosIKBodyCoordinate(0,140,50,-80);
      SetPosIKBodyCoordinate(1,-140,50,-80);
      SetPosIKBodyCoordinate(2,-140,-50,-80);
      SetPosIKBodyCoordinate(3,140,-50,-80);
      SetAnglesFromArray(AnglesIK);
  //    SetAnglesFromArray(Angles1);
      }
    else if(str1=="2")
      {
        //SetAnglesFromArray(Angles2);
//        SetPosIKLegCoordinate(0,90,0,-60);
//        SetPosIKLegCoordinate(1,90,0,-60);
//        SetPosIKLegCoordinate(2,90,0,-60);
//        SetPosIKLegCoordinate(3,90,0,-60);
        SetPosIKBodyCoordinate(0,50,140,-50);
        SetPosIKBodyCoordinate(1,-50,140,-50);
        SetPosIKBodyCoordinate(2,-50,-140,-50);
        SetPosIKBodyCoordinate(3,50,-140,-50);
        SetAnglesFromArray(AnglesIK);
//      SetAnglesFromArray(Angles2);
      }
    else if(str1=="3")
      {
      SetPosIKLegCoordinate(0,90,20,-60);
      SetPosIKLegCoordinate(1,90,20,-60);
      SetPosIKLegCoordinate(2,90,20,-60);
      SetPosIKLegCoordinate(3,90,20,-60);
        SetAnglesFromArray(AnglesIK);
//      SetAnglesFromArray(Angles3);
      }
    else if(str1=="4")
    {
      SetAnglesFromArray(Angles4);
    ;}
    else if(str1=="5")
    {;}
    else if(str1=="6")
    {;}
    else if(str1=="w")
    {
      WalkTest();
    }
    else 
      SetAnglesFromArray(initialAngles);

    
  }

  
}
