void BackWalkSideway(int repetitions)
{
    logPrintln("Back Walk Sideway Start");
    SidewayGait ICrawl; // 間歇クロール歩容のクラスインスタンス
    ICrawl.SetFootBaseDefault(); // 歩容の基準姿勢
    ICrawl.Update_Back(0, &robotState); // 歩行の基準姿勢
    delay(500);

    for (int r = 0; r < repetitions; r++) {
        for (int t = 0; t < 100; t++) {
            float phase = (float)t / 100.0; // 0から1までの値を計算
            ICrawl.Update_Back(phase, &robotState);
            SetAnglesFromState(robotState); // 最終的な姿勢をServoに反映
            delay(20); // 各ステップごとの待機時間（ミリ秒）
        }
    }

    ICrawl.Update_Back(0, &robotState); // 最後に止まる
    SetAnglesFromState(robotState);

    logPrintln("Back Walk  Sideway End");
}

// CMD
void cmd_BackWalkSidway(RobotState* s, int n, int)  { BackWalkSideway(n); }
