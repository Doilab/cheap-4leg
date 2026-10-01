void WalkIC(int repetitions) //間歇クロールによる前進歩行
{
    //引数を繰り返し回数(repetitions)に変更
    logPrintln("Walk Start");
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

    logPrintln("ICrawl Walk End");
}

// CMD
void cmd_WalkIC(RobotState* s, int n, int)  { WalkIC(n); }
