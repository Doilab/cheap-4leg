//#define GLM_FORCE_PURE
//#define GLM_FORCE_SINGLE_ONLY

#include <glm/glm.hpp>//ベクトルや行列の計算に使うライブラリ

#include <Arduino.h>//数学関数もここに入っている
#include <M5Unified.h>

//#include <math.h>//マクロ汚染されるので危険

// --- ここからWi-Fi設定 ---
#include <WiFi.h>
#include <WebServer.h>

#include "config.h"

WebServer server(80);


//サーボ
#include "movement/servo.h"

//運動学
#include "movement/kinematics.h"

//診断用関数群
#include "diagnosis.h"

//モーション関数群
#include "movement/motion.h"

//トロット歩容
#include "movement/trot_gait.h"

//間歇クロール歩容
#include "movement/intermittent_crawl_gait.h"

RobotState robotState; // ロボットの状態を保持する構造体
String str_robot_name; // ロボット名を保持する変数

// Server Include
#include "server/html_server.h"

//---------------------------------------------
void SetAnglesFromState(RobotState state)
{
  //RobotStateから関節角度（Leg角）を取り出して駆動
  //全関節
  for (int leg = 0; leg < 4; ++leg) {
    moveLegWithOffset(leg, state.legs[leg].jointAngles[0], state.legs[leg].jointAngles[1], state.legs[leg].jointAngles[2]);
  }
}
//---------------------------------------------
char SetFootPosIKBodyCoordinate(int legIndex, glm::vec3 pos)
{
  char flag = SetFootPosIKBodyCoordinateToRobotState(legIndex, pos, &robotState);
  return flag;
}

//---------------------------------------------
void InitStatus(RobotState *state)
{
  SetInitialPose(state);
  SetAnglesFromState(*state);
}
//---------------------------------------------
void Bowing(RobotState *state)
{
  //お辞儀をする関数
  Serial.println("Bowing motion start..." );

  RobotState start_r = *state;//初期姿勢
  RobotState target_r = *state;//目標姿勢（お辞儀姿勢）

  target_r.legs[0].footPos.z += 30;//脚先を30mm上げる
  target_r.legs[3].footPos.z += 30;//脚先を30mm上げる

  glm::vec3 pos;
  float phase = 0.0;

  for (phase = 0.0; phase <=1.0; phase+=0.01) {
    if(phase<=0.4)
    {
      // --- A. お辞儀をする動作） ---
      Serial.println("Bowing...");
      float ratio = phase / 0.4;
      for (int leg = 0; leg < 4; leg++) {
        glm::vec3 s = start_r.legs[leg].footPos;
        glm::vec3 t = target_r.legs[leg].footPos;
        pos = s + (t - s) * ratio;//RobotStateの脚先位置も補正して線形補間する． 
        SetFootPosIKBodyCoordinateToRobotState(leg, pos, state);
      }
    }
    else if(phase<=0.6)
    {
      // --- B. 停止 ---
      Serial.println("Bowing keep...");
      for (int leg = 0; leg < 4; leg++) {
        pos = target_r.legs[leg].footPos;
        SetFootPosIKBodyCoordinateToRobotState(leg, pos, state);
      }
    }
    else if(phase<=1.0)
    {
      // --- C. 元の体勢に戻る動作 ---
      Serial.println("Bowing return...");
      float ratio = (phase-0.6) / 0.4;
      for (int leg = 0; leg < 4; leg++) {
        glm::vec3 t = target_r.legs[leg].footPos;
        glm::vec3 s = start_r.legs[leg].footPos;
        pos = t + (s - t) * ratio;//RobotStateの脚先位置も補正して線形補間する． 
        SetFootPosIKBodyCoordinateToRobotState(leg, pos, state);
      }
    }

    SetAnglesFromState(*state); // RobotStateからServoに反映
    delay(25); 
  }
  Serial.println("Bowing done.");
  //delay(25); 

  // --- ここまで ---
}
//---------------------------------------------

//----------------------------------------------
void WalkTrot(int repetitions, int RotateMode) //トロット歩容による前進歩行
{
  Serial.println("Trot Walk Start");
  TrotGait  Trot; // トロット歩容のクラスインスタンス
  
  //Trot.SetFootBaseDefault(); // 歩容の基準姿勢をセット
  Trot.SetFootBase(&robotState); // RobotStateから歩容の基準姿勢をセット．
  Trot.Update(0, RotateMode, &robotState); // 歩行の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (float t = 0; t < 100; t++) {
      float phase = t / 100.0; // 0から1までの値を計算
      Trot.Update(phase, RotateMode, &robotState); // 0から1までの値をTrotに渡す
      SetAnglesFromState(robotState); // RobotStateからServoに反映

      delay(20); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  Trot.Update(0, RotateMode, &robotState); // 最後に止まる
  SetAnglesFromState(robotState); // 最終的な姿勢をServoに反映

  Serial.println("Trot Walk End");
}
//----------------------------------------------
void WalkIC(int repetitions) //間歇クロールによる前進歩行
{
//引数を繰り返し回数(repetitions)に変更
  Serial.println("Walk Start");
  IntermittentCrawlGait ICrawl; // 間歇クロール歩容のクラスインスタンス
  ICrawl.SetFootBaseDefault(); // 歩容の基準姿勢
  ICrawl.Update(0, &robotState); // 歩行の基準姿勢
  //ICrawl(0, &robotState); // 歩行の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (int t = 0; t < 100; t++) {
      float phase = (float)t / 100.0; // 0から1までの値を計算
      ICrawl.Update(phase, &robotState); 
      //ICrawl(phase, &robotState); // 0から1までの値をICrawl2に渡す
      SetAnglesFromState(robotState); // 最終的な姿勢をServoに反映
      delay(20); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  ICrawl.Update(0, &robotState); // 最後に止まる
  //ICrawl(0, &robotState); // 最後に止まる
  SetAnglesFromState(robotState);

  Serial.println("ICrawl Walk End");
}
//----------------------------------------------
void BackWalkIC(int repetitions) //間歇クロールによる後退歩行
{
  Serial.println("Back Walk Start");
  IntermittentCrawlGait ICrawl; // 間歇クロール歩容のクラスインスタンス
  ICrawl.SetFootBaseDefault(); // 歩容の基準姿勢
  ICrawl.Update_Back(0, &robotState); // 歩行の基準姿勢
  //ICrawl_Back(0, &robotState); // 後退の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (double t = 0; t < 100; t++) {
      //ICrawl_Back(t/100.0, &robotState); // 0から1までの値をICrawl2に渡す
      ICrawl.Update_Back(t/100.0, &robotState); 
      SetAnglesFromState(robotState); // 最終的な姿勢をServoに反映
      delay(20); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  //ICrawl_Back(0,&robotState); // 最後に止まる
  ICrawl.Update_Back(0, &robotState); // 最後に止まる
  SetAnglesFromState(robotState); // RobotStateからServoに反映
  Serial.println("ICrawl Back End");
}

//---------------------------------------------

//Include server files.
#include "server/server.h"
//------------------------------------------------------------
//---------------------------------------------
void setup() {
  str_robot_name = ROBOT_NAME;
  Serial.begin(115200);
  Serial.println("--- Booting Robot ---");
  delay(1000); // 起動直後に少し待機
  
  
  // 先にWi-Fiを立ち上げる
  Serial.println("--- Initializing Wi-Fi ---");
  setupWiFi(); 

  // その後にサーボなどの設定をする
  Serial.println("--- Initializing Servos ---");
  initServos();
  
  Serial.println("System Ready.");
  //ICrawl(0,&robotState); // 間歇クロール歩容の基準姿勢へ
  WalkIC(0); // 歩行の基準姿勢へ
  free_all(); // 脱力状態へ
}

//---------------------------------------------
void loop() {
  // 常にWebサービス（スマホ）からのアクセスをチェック（止めてはいけない）
  server.handleClient();

  // USBシリアルからの入力があるときだけ、以下の処理を行う
  if (Serial.available() > 0) {
    String str1 = Serial.readString();
    str1.trim();
    Serial.println("Received: " + str1);

    if (str1 == "0") {
      free_all();
    } else if (str1 == "1") {
      PWM_test();
    } else if (str1 == "2") {
      moveServo_test();
    } else if (str1 == "3") {
      moveLegWithOffset_test();
    } else if (str1 == "4") {
      SetFootPosIKLegCoordinate_test();
    } else if (str1 == "5") {
      SetFootPosIKBodyCoordinate_test();
    } else if (str1.startsWith("w")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        WalkIC(count);
      } else {
        Serial.println("Reset Forward Pose");
        //ICrawl(0,&robotState);
        WalkIC(0);
      }
    } else if (str1.startsWith("b")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        BackWalkIC(count);
      } else {
        Serial.println("Reset Backward Pose");
        //ICrawl_Back(0,&robotState);
        BackWalkIC(0);
      }
    } else if (str1 == "h") {
      Serial.println("Hello!");
      Bowing(&robotState);
    } else if (str1 == "t") {
      Serial.println("Trot");
      WalkTrot(1, 1); // 1回繰り返し、RotateMode=1（左回り）
    } else
    {
      InitStatus(&robotState); // RobotStateの初期化
      Serial.println("Unknown command. Reset to initial pose.");
    }
    
    // 次の命令を促す表示（シリアル入力があった時だけ出す）
    String str2 = "-- " + str_robot_name + " --\n";
    Serial.println(str2);
    Serial.println("0:Free, 1:PWM, 2:moveservo, 3:SetMotors, 4:IKLeg, 5:IKBody, w:walk, b:back, t:trot, h:hello");
  }

  // ループが速すぎると通信が不安定になることがあるため、ごくわずかに待機
  delay(10);
}
//---------------------------------------------
