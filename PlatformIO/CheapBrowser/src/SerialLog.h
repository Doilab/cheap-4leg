#ifndef SERIAL_LOG_H
#define SERIAL_LOG_H
#include <Arduino.h>
#include <freertos/queue.h>

#define LOG_MSG_LEN 128
#define LOG_QUEUE_SIZE 100

extern QueueHandle_t logQueue;

inline void logInit() {
  logQueue = xQueueCreate(LOG_QUEUE_SIZE, LOG_MSG_LEN);
}

inline void logPrintln(const char* msg) {
  char buf[LOG_MSG_LEN];
  strncpy(buf, msg, LOG_MSG_LEN - 1);
  buf[LOG_MSG_LEN - 1] = '\0';
  xQueueSend(logQueue, buf, 0);
}

inline void logPrintln(const String& msg) {
  logPrintln(msg.c_str());
}

inline void logFlush(int max_messages = 100) {
  char buf[LOG_MSG_LEN];
  int count = 0;
  while (count < max_messages && xQueueReceive(logQueue, buf, 0)) {
    Serial.println(buf);
    count++;
  }
}

#endif
