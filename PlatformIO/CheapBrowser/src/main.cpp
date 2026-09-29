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

// Init CMD Handler
QueueHandle_t cmdQueue;

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
// Importing movement Command Structure
//---------------------------------------------
#include "move_cmd/cmd_strc.h"
//---------------------------------------------
// Importing movement Commands
//---------------------------------------------
#include "move_cmd/Bowing.h"
//----------------------------------------------
#include "move_cmd/WalkTrot.h"
//----------------------------------------------
#include "move_cmd/WalkIC.h"
//----------------------------------------------
#include "move_cmd/BackWalkIC.h"
//---------------------------------------------

//Include server files.
#include "server/server.h"
//---------------------------------------------
// --- Motor Task ---
void motorTask(void*) {
  MotionCmd cmd;
  while(1) {
    if (xQueueReceive(cmdQueue, &cmd, portMAX_DELAY)) {
      if (cmd.fn != nullptr) {  // <-- sicher
        cmd.fn(&robotState, cmd.arg1, cmd.arg2);
      }
    }
  }
}
//--------------------------------------------
void setup() {
  str_robot_name = ROBOT_NAME;
  Serial.begin(115200);
  Serial.println("--- Booting Robot ---");
  delay(1000); // 起動直後に少し待機
   
  // Init Task Queue
  cmdQueue = xQueueCreate(5, sizeof(MotionCmd));
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, NULL, 1, NULL, 0);
  
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

    MotionCmd cmd;

    if (str1 == "0") {
      cmd = {cmd_free};
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
        cmd = {cmd_WalkIC, count};
      } else {
        Serial.println("Reset Forward Pose");
        //ICrawl(0,&robotState);
        cmd = {cmd_WalkIC, 0};
      }
    } else if (str1.startsWith("b")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        cmd = {cmd_BackWalkIC, count};
      } else {
        Serial.println("Reset Backward Pose");
        //ICrawl_Back(0,&robotState);
        cmd = {cmd_BackWalkIC, 0};
      }
    } else if (str1 == "h") {
      Serial.println("Hello!");
      cmd = {cmd_Bowing};
    } else if (str1 == "t") {
      Serial.println("Trot");
      cmd = {cmd_WalkTrot, 1, 1}; // 1回繰り返し、RotateMode=1（左回り）
    } else
    {
      cmd = {cmd_InitStatus}; // RobotStateの初期化
      Serial.println("Unknown command. Reset to initial pose.");
    }

    // Executing CMD
    xQueueSend(cmdQueue, &cmd, 0);
    
    // 次の命令を促す表示（シリアル入力があった時だけ出す）
    String str2 = "-- " + str_robot_name + " --\n";
    Serial.println(str2);
    Serial.println("0:Free, 1:PWM, 2:moveservo, 3:SetMotors, 4:IKLeg, 5:IKBody, w:walk, b:back, t:trot, h:hello");
  }

  // ループが速すぎると通信が不安定になることがあるため、ごくわずかに待機
  delay(10);
}
//---------------------------------------------
