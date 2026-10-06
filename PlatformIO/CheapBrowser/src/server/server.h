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
  server.on("/fight", handleFightView);
  server.on("/home",  []() { MotionCmd c={cmd_WalkIC,0};      xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//間歇クロールの初期状態へ
  server.on("/wf",    []() { MotionCmd c={cmd_WalkIC,1};      xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//歩行開始関数を呼び出す
  server.on("/wb",    []() { MotionCmd c={cmd_BackWalkIC,1};  xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//後退関数を呼び出す
  server.on("/tr",    []() { MotionCmd c={cmd_WalkTrot,1,-1}; xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//トロット右ターン
  server.on("/tl",    []() { MotionCmd c={cmd_WalkTrot,1,1};  xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//トロット左ターン
  server.on("/tf",    []() { MotionCmd c={cmd_WalkTrot,1,0};  xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//トロット前進

  // Fight Commands
  server.on("/klr",    []() 
    {  
      MotionCmd cmd = {cmd_Kick_Left_Right};
      xQueueSend(cmdQueue, &cmd, 0);
      cmd = {cmd_BackWalkIC, 0};
      xQueueSend(cmdQueue, &cmd, 0);
      server.send(200, "text/plain", "OK"); 
    }
  );//トロット左ターン
  
  server.on("/kst",    []() 
    {  
      MotionCmd cmd = {cmd_Kick_Front};
      xQueueSend(cmdQueue, &cmd, 0);
      cmd = {cmd_BackWalkIC, 0};
      xQueueSend(cmdQueue, &cmd, 0);
      server.send(200, "text/plain", "OK"); 
    }
  );//トロット前進

  server.on("/bow",   []() { MotionCmd c={cmd_Bowing};        xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//お辞儀
  server.on("/i",     []() { MotionCmd c={cmd_InitStatus};    xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//初期化姿勢（足を伸ばした状態）
  server.on("/0",     []() { MotionCmd c={cmd_free};          xQueueSend(cmdQueue,&c,0); server.send(200, "text/plain", "OK"); });//脱力
  

  server.begin();
  Serial.println("HTTP Server Started");
  Serial.println("--- Wi-Fi Setup Done ---");
}
