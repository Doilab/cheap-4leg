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

// CMD
void cmd_Bowing(RobotState* s, int, int)    { Bowing(s); }
