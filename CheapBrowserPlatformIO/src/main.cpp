#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>

// --- ここからWi-Fi設定 ---
#include <WiFi.h>
#include <WebServer.h>

const char* ap_ssid = "M5Atom_Robot_test";//他の人と違うssidにする
const char* ap_pass = "12345678";
WebServer server(80);

//サーボと運動学
#include "servo_kinematics.h"

//診断用関数群
#include "diagnosis.h"

//モーション関数群
#include "motion.h"


//関節角度配列 逆運動学結果格納用(Leg角)
float AnglesIK[4][3] = {
  {0.0, 0.0, 0.0}, // LEG1
  {0.0, 0.0, 0.0}, // LEG2
  {0.0, 0.0, 0.0}, // LEG3
  {0.0, 0.0, 0.0}, // LEG4
};


//---------------------------------------------
void SetFootPosIKBodyCoordinate(int legIndex, double x, double y, double z)
{
  SetFootPosIKBodyCoordinateToArray(legIndex, x, y, z, AnglesIK);
}


//---------------------------------------------
void Bowing(void)
{
  //お辞儀をする関数
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
      SetFootPosIKBodyCoordinate(i, x, y, z);
      //FootPos[i][0] = x; FootPos[i][1] = y; FootPos[i][2] = z;
    }
    SetAnglesFromArray(AnglesIK);
    delay(25); 
  }

  // --- B. 1秒間停止 ---
  Serial.println("Web: 2sec wait...");
  delay(1000); 

  // --- C. 元の体勢に戻る動作（スロー） ---
  Serial.println("Web: return");
  for (int s = 1; s <= steps; s++) {
    float ratio = (float)s / (float)steps;
    for (int i = 0; i < 4; i++) {
      // targetからstartへ向かって計算
      float x = target[i][0] + (start[i][0] - target[i][0]) * ratio;
      float y = target[i][1] + (start[i][1] - target[i][1]) * ratio;
      float z = target[i][2] + (start[i][2] - target[i][2]) * ratio;
      SetFootPosIKBodyCoordinate(i, x, y, z);
      //FootPos[i][0] = x; FootPos[i][1] = y; FootPos[i][2] = z;
    }
    SetAnglesFromArray(AnglesIK);
    delay(25); 
  }
  Serial.println("Web: bowing done.");
  // --- ここまで ---
}
//---------------------------------------------

//----------------------------------------------
void WalkTrot(int repetitions, int RotateMode) //トロット歩容による前進歩行
{
  Serial.println("Trot Walk Start");
  //char RotateMode = 1; // 胴体回転の有無を制御するフラグ（1: 左回り，-1:右回り）
  Trot(0, RotateMode, AnglesIK); // 歩行の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (double t = 0; t < 100; t++) {
      Trot(t/100.0, RotateMode, AnglesIK); // 0から1までの値をTrotに渡す
      SetAnglesFromArray(AnglesIK);
      delay(50); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  Trot(0, RotateMode, AnglesIK); // 最後に止まる
  SetAnglesFromArray(AnglesIK);

  Serial.println("Trot Walk End");
}
//----------------------------------------------
void WalkIC(int repetitions) //間歇クロールによる前進歩行
{
//引数を繰り返し回数(repetitions)に変更
  Serial.println("Walk Start");
  ICrawl(0, AnglesIK); // 歩行の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (double t = 0; t < 100; t++) {
      ICrawl(t/100.0,AnglesIK); // 0から1までの値をICrawl2に渡す
      SetAnglesFromArray(AnglesIK);
      delay(50); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  ICrawl(0, AnglesIK); // 最後に止まる
  SetAnglesFromArray(AnglesIK);

  Serial.println("ICrawl Walk End");
}
//----------------------------------------------
void BackWalkIC(int repetitions) //間歇クロールによる後退歩行
{
  Serial.println("Back Walk Start");
  ICrawl_Back(0,AnglesIK); // 後退の基準姿勢
  delay(500);

  for (int r = 0; r < repetitions; r++) {
    for (double t = 0; t < 100; t++) {
      ICrawl_Back(t/100.0,AnglesIK); // 0から1までの値をICrawl2に渡す
      SetAnglesFromArray(AnglesIK);
      delay(50); // 各ステップごとの待機時間（ミリ秒）
    }
  }

  ICrawl_Back(0,AnglesIK); // 最後に止まる
  SetAnglesFromArray(AnglesIK);
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
  html += "  <button class='c-red' onclick=\"fetch('/w')\">↑F</button>";
  html += "  <button class='c-green' onclick=\"fetch('/reset')\">Reset</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/b')\">↓B</button>";
  html += "  <button class='c-green' onclick=\"fetch('/lt')\">←L</button>";
  html += "  <button class='c-green' onclick=\"fetch('/reset')\">Trot</button>";
  html += "  <button class='c-green' onclick=\"fetch('/rt')\">R→</button>";
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
  server.on("/reset", []() { ICrawl(0,AnglesIK); server.send(200, "text/plain", "OK"); });//間歇クロールの初期状態へ
  server.on("/w", []() { WalkIC(1); server.send(200, "text/plain", "OK"); });//歩行開始関数を呼び出す
  server.on("/b", []() { BackWalkIC(1); server.send(200, "text/plain", "OK"); });//後退関数を呼び出す
  server.on("/h", []() { Bowing(); server.send(200, "text/plain", "OK"); });//お辞儀
  server.on("/rt", []() { WalkTrot(1,-1); server.send(200, "text/plain", "OK"); });//右ターン
  server.on("/lt", []() {  WalkTrot(1,1); server.send(200, "text/plain", "OK"); });//左ターン


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
  init_servos();
  
  Serial.println("System Ready.");
  ICrawl(0,AnglesIK); // 間歇クロール歩容の基準姿勢へ
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
        ICrawl(0,AnglesIK);
      }
    } else if (str1.startsWith("b")) {
      if (str1.length() > 1) {
        int count = str1.substring(1).toInt();
        if (count <= 0) count = 1;
        BackWalkIC(count);
      } else {
        Serial.println("Reset Backward Pose");
        ICrawl_Back(0,AnglesIK);
      }
    } else if (str1 == "h") {
      Serial.println("Hello!");
      Bowing();
    } else if (str1 == "t") {
      Serial.println("Trot");
      WalkTrot(1, 1); // 1回繰り返し、RotateMode=1（左回り）
    } else
    {
      //SetAnglesFromArray(initialAngles);
      SetInitialPose(AnglesIK);
      SetAnglesFromArray(AnglesIK);
      Serial.println("Unknown command. Reset to initial pose.");
    }
    
    // 次の命令を促す表示（シリアル入力があった時だけ出す）
    Serial.println("1:PWM, 2:moveservo, 3:array, 4:IKLeg, 5:IKBody, w:walk, b:back, t:trot, h:hello");
  }

  // ループが速すぎると通信が不安定になることがあるため、ごくわずかに待機
  delay(10);
}
//---------------------------------------------