#ifndef __CONFIG_H__
#define __CONFIG_H__


#define ROBOT_NO 2

#if ROBOT_NO == 0
  #define ROBOT_NAME "Robot_No.0"
  #define AP_SSID "C4LRobot_No0"
#elif ROBOT_NO == 1
  #define ROBOT_NAME "Robot_No.1"
  #define AP_SSID "C4LRobot_No1"
#elif ROBOT_NO == 2
  #define ROBOT_NAME "Robot_No.2"
  #define AP_SSID "C4LRobot_No2"
#elif ROBOT_NO == 3
  #define ROBOT_NAME "Robot_No.3"
  #define AP_SSID "C4LRobot_No3"
#elif ROBOT_NO == 4
  #define ROBOT_NAME "Robot_No.4"
  #define AP_SSID "C4LRobot_No4"
#elif ROBOT_NO == 5
  #define ROBOT_NAME "Robot_No.5"
  #define AP_SSID "C4LRobot_No5"
#else
  #define ROBOT_NAME "Robot_test"
  #define AP_SSID "Robot_test"
#endif

const char* ap_ssid = AP_SSID;
const char* ap_pass = "12345678";

//補正配列．あまり値が大きくならないようにできるだけ組立時に合わせておく．
//ロボットの機体により個体差あり
#if ROBOT_NO == 0
//0号
float OffsetAngles[4][3] = {
  {-10.0, 0.0, 0.0}, // LEG1
  {-7.0, -5.0, 15.0}, // LEG2
  {0.0, -10.0, 0.0}, // LEG3
  {-10.0, 0.0, -5.0}  // LEG4
};
#elif ROBOT_NO == 1
//黒１号
float OffsetAngles[4][3] = {
  {12.0, 0.0, 15.0}, // LEG1
  {12.0, 0.0, -5.0}, // LEG2
  {0.0, 5.0, 5.0}, // LEG3
  {0.0, 8.0,5.0}  // LEG4
};
#elif ROBOT_NO == 2
//黒グレー２号
float OffsetAngles[4][3] = {
  {20.0, 0.0, 0.0}, // LEG1
  {12.0, 5.0, 0.0}, // LEG2
  {-10.0, 0.0, 0.0}, // LEG3
  {-10.0, 0.0, 5.0}  // LEG4
};
#elif ROBOT_NO == 3
//黒３号
float OffsetAngles[4][3] = {
  {0.0, 0.0, -5.0}, // LEG1
  {0.0, 0.0, 0.0}, // LEG2
  {0.0, 5.0, 5.0}, // LEG3
  {0.0, 15.0, -5.0}  // LEG4
};
#elif ROBOT_NO == 4
//黒４号
float OffsetAngles[4][3] = {
  {0.0, 10.0, -5.0}, // LEG1
  {0.0, 20.0, -5.0}, // LEG2
  {5.0, 5.0, 0.0}, // LEG3
  {0.0, -10.0, -15.0}  // LEG4
};
#elif ROBOT_NO == 5
//黒５号
float OffsetAngles[4][3] = {
  {0.0, 15.0, 0.0}, // LEG1
  {0.0, 10.0, -15.0}, // LEG2
  {0.0, 20.0, -5.0}, // LEG3
  {0.0, -5.0, 0.0}  // LEG4
};
#else
//デフォルト
float OffsetAngles[4][3] = {
  {0.0, 0.0, 0.0}, // LEG1
  {0.0, 0.0, 0.0}, // LEG2
  {0.0, 0.0, 0.0}, // LEG3
  {0.0, 0.0, 0.0}  // LEG4
};
#endif

#endif // __CONFIG_H__
