// Minimal host stand-ins for the OpenCatEsp32 / Arduino symbols flyEscape.h uses, so the
// module can be syntax- and type-checked on a laptop. NOT the real firmware.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#define INPUT 1
#define ANALOG3 36
#define ANALOG4 39
static unsigned long fake_ms = 0;
static int fake_adc[40];
inline unsigned long millis() { return fake_ms; }
inline void delay(unsigned long ms) { fake_ms += ms; }
inline void pinMode(int, int) {}
inline int analogRead(int pin) { return fake_adc[pin]; }
static std::string fake_out;  // everything printed to "all ports", one line each
inline void printToAllPorts(const char *text) {
  fake_out += text;
  fake_out += "\n";
  std::printf("%s\n", text);
}
struct FakeQueue {
  int n = 0;
  int adds = 0;
  int size() { return n; }
  template <typename T>
  void addTask(char, T *, int = 0) {
    adds++;
    n++;
  }
};
static FakeQueue fakeQueue;
static FakeQueue *tQueue = &fakeQueue;
