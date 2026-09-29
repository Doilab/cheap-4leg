struct MotionCmd {
    void (*fn)(RobotState*, int, int);
    int arg1 = 0;  // default 0
    int arg2 = 0;  // default 0
                   //
    MotionCmd() : fn(nullptr), arg1(0), arg2(0) {}             
    MotionCmd(void (*f)(RobotState*, int, int), int a1 = 0, int a2 = 0)
        : fn(f), arg1(a1), arg2(a2) {}
};

void cmd_InitStatus(RobotState* s, int, int) { InitStatus(s); }
void cmd_free(RobotState* s, int, int)      { free_all(); }
