void AngleMove(float Angles[4][3]) {
  char buf[64];
  logPrintln("");
  for (int leg = 0; leg < 4; leg++) {
    float x = Angles[leg][0];
    float y = Angles[leg][1];
    float z = Angles[leg][2];
    moveLegWithOffset(leg, x, y, z);
    sprintf(buf, "Leg %d angles (%.1f, %.1f, %.1f)", leg, x, y, z);
    logPrintln(buf);
  }
}
