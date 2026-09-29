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

// CMD
void cmd_BackWalkIC(RobotState* s, int n, int)  { BackWalkIC(n); }
