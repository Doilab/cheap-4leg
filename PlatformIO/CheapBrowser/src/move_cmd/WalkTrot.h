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

// CMD 
void cmd_WalkTrot(RobotState* s, int n, int rot) { WalkTrot(n, rot); }
