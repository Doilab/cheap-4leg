//モーションを記述する
#ifndef MOTION_H
#define MOTION_H
//#include <math.h>
#include "kinematics.h"
#include <gtc/matrix_transform.hpp> //ArduinoIDE
//#include <glm/gtc/matrix_transform.hpp>
//---------------------------------------------

//---------------------------------------------
void SetInitialPose(RobotState *state_out)
{
  //初期姿勢にする関数
  //RobotState構造体初期化
  for (int leg = 0; leg < 4; ++leg) {
    state_out->legs[leg].jointAngles[0] = 0.0;
    state_out->legs[leg].jointAngles[1] = 0.0;
    state_out->legs[leg].jointAngles[2] = 0.0;
    //脚先位置は関節角度から計算して入れる．
    //↓未実装
    //float x, y, z;
    // calcFK_Leg(state_out->legs[leg].jointAngles[0], state_out->legs[leg].jointAngles[1], state_out->legs[leg].jointAngles[2], &x, &y, &z);
    // state_out->legs[leg].footPos = glm::vec3(x, y, z);
  }
}
//---------------------------------------------

#endif