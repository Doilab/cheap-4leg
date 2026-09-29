struct MotionCmd {
    void (*fn)(RobotState*, int, int);
    int arg1 = 0;  // default 0
    int arg2 = 0;  // default 0
};

void cmd_InitStatus(RobotState* s, int, int) { InitStatus(s); }
void cmd_free(RobotState* s, int, int)      { free_all(); }
