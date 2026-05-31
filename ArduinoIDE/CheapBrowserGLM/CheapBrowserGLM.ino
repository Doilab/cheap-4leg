//#define GLM_FORCE_PURE
//#define GLM_FORCE_SINGLE_ONLY

//#include <glm/glm.hpp>//ベクトルや行列の計算に使うライブラリ
#include <glm.hpp>//ベクトルや行列の計算に使うライブラリ.ArduinoIDEの場合

#include <Arduino.h>//数学関数もここに入っている
#include <M5Unified.h>

//#include <math.h>//マクロ汚染されるので危険

// --- ここからWi-Fi設定 ---
#include <WiFi.h>
#include <WebServer.h>

const char* ap_ssid = "M5Atom_Robot_test";//他の人と違うssidにする
const char* ap_pass = "12345678";
WebServer server(80);

//サーボ
#include "servo.h"

//運動学
#include "kinematics.h"

//診断用関数群
#include "diagnosis.h"

//モーション関数群
#include "motion.h"

//トロット歩容
#include "trot_gait.h"

//間歇クロール歩容
#include "intermittent_crawl_gait.h"

RobotState robotState; // ロボットの状態を保持する構造体

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

      delay(50); // 各ステップごとの待機時間（ミリ秒）
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
      delay(50); // 各ステップごとの待機時間（ミリ秒）
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
      delay(50); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  //ICrawl_Back(0,&robotState); // 最後に止まる
  ICrawl.Update_Back(0, &robotState); // 最後に止まる
  SetAnglesFromState(robotState); // RobotStateからServoに反映
  Serial.println("ICrawl Back End");
}

//---------------------------------------------
// スマホに表示される操作画面
void handleRoot() {
  String html = "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
  html += "<style>";
  // 背景設定
  html += "body { background-color: #2d2d2d; color: white; font-family: sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; margin: 0; overflow: hidden; }";
  html += "h1 { font-size: 28px; margin-bottom: 25px; letter-spacing: 2px; }";
  
  // グリッド配置
  html += ".grid { display: grid; grid-template-columns: repeat(3, 80px); grid-gap: 30px; }";
  
  // 基本ボタン設定（太い黒枠の白い正方形）
  html += "button { width: 100px; height: 100px; background-color: white; border: 8px solid #000; position: relative; font-size: 24px; font-weight: bold; cursor: pointer; transition: 0.1s; display: flex; align-items: center; justify-content: center; z-index: 1; }";
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

  html += "<h1>Cheap 4Leg Robot</h1>";
  
  html += "<div class='grid'>";
  // 各ボタン（classで色を呼び出し）
  html += "  <button class='c-blue' onclick=\"fetch('/w')\">↑F</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/reset')\">Reset</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/b')\">↓B</button>";
  html += "  <button class='c-green' onclick=\"fetch('/lt')\">←CCW</button>";
  html += "  <button class='c-green' onclick=\"fetch('/t')\">Trot</button>";
  html += "  <button class='c-green' onclick=\"fetch('/rt')\">CW→</button>";
  html += "  <button class='c-red' onclick=\"fetch('/i')\">Init</button>";
  html += "  <button class='c-yellow' onclick=\"fetch('/h')\">お辞儀</button>";
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
  server.on("/reset", []() { WalkIC(0); server.send(200, "text/plain", "OK"); });//間歇クロールの初期状態へ
  server.on("/w", []() { WalkIC(1); server.send(200, "text/plain", "OK"); });//歩行開始関数を呼び出す
  server.on("/b", []() { BackWalkIC(1); server.send(200, "text/plain", "OK"); });//後退関数を呼び出す
  server.on("/rt", []() { WalkTrot(1,-1); server.send(200, "text/plain", "OK"); });//トロット右ターン
  server.on("/lt", []() {  WalkTrot(1,1); server.send(200, "text/plain", "OK"); });//トロット左ターン
  server.on("/t", []() {  WalkTrot(1,0); server.send(200, "text/plain", "OK"); });//トロット前進

  server.on("/h", []() { Bowing(&robotState); server.send(200, "text/plain", "OK"); });//お辞儀
  server.on("/i", []() { InitStatus(&robotState); server.send(200, "text/plain", "OK"); });//初期化姿勢（足を伸ばした状態）
  

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
  initServos();
  
  Serial.println("System Ready.");
  //ICrawl(0,&robotState); // 間歇クロール歩容の基準姿勢へ
  WalkIC(0); // 歩行の基準姿勢へ
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
    Serial.println("1:PWM, 2:moveservo, 3:SetMotors, 4:IKLeg, 5:IKBody, w:walk, b:back, t:trot, h:hello");
  }

  // ループが速すぎると通信が不安定になることがあるため、ごくわずかに待機
  delay(10);
}
//---------------------------------------------