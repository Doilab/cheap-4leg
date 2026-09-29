// Webブラウザに表示される操作画面
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

  //html += "<h1>Cheap 4Leg Robot</h1>";
  html += "<h1>== " + str_robot_name + " ==</h1>";
  
  html += "<div class='grid'>";
  // 各ボタン（classで色を呼び出し）
  html += "  <button class='c-blue' onclick=\"fetch('/wf')\">↑F</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/home')\">Home</button>";
  html += "  <button class='c-blue' onclick=\"fetch('/wb')\">↓B</button>";
  html += "  <button class='c-green' onclick=\"fetch('/tl')\">←CCW</button>";
  html += "  <button class='c-green' onclick=\"fetch('/tf')\">Trot</button>";
  html += "  <button class='c-green' onclick=\"fetch('/tr')\">CW→</button>";
  html += "  <button class='c-red' onclick=\"fetch('/i')\">Init</button>";
  html += "  <button class='c-yellow' onclick=\"fetch('/bow')\">お辞儀</button>";
  html += "  <button class='c-red' onclick=\"fetch('/0')\">Free</button>";
  html += "</div>";

  html += "<div class='footer'>M5Atom S3 Controller</div>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}


