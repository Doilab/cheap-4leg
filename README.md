# cheap-4leg
チープ４足ロボットのプログラム

## プログラム
* cheap1 --- シンプルなプログラム．WiFi不使用
* CheapBrowser --- WiFi経由でブラウザでコントロールできる．シリアルインターフェースもある


## Arduino IDE による開発環境設定
* ボードマネージャから以下インストール
    - ESP32 by Espressif Systems
* ボード　M5atom S3 を選ぶ
* ライブラリマネージャから以下インストール
    - Adafruit PWM Servo Driver Library
    - Adafruit BUS IO

## ブラウザでの使い方
* SSIDに接続　"M5Atom_Robot_test"など．
* パスワード入力
* 192.168.4.1をブラウザで開くと以下の操縦画面が開く．

<img src="screenshot.png">

## シリアルインターフェースでの使い方
* シリアルモニタ等で115200bpsで接続
* w+回数で間歇クロールを回数周期だけ実行
* （cheap1の場合　wキー+Enter1回で間歇クロール待機状態， ここで何かキーを押すと歩行開始）
* 1 + Enter で PWMテスト
* 2 + Enter  で サーボ角テスト
* 3 + Enter  で Leg角配列テスト
* 4 + Enter  で 脚座標系で逆運動学テスト
* 5 + Enter  で 胴体座標系で逆運動学テスト
