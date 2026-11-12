// Minimal host stand-ins for the OpenCatEsp32 / Arduino symbols flyEscape.h uses, so the
// module can be syntax- and type-checked on a laptop. NOT the real firmware.
#pragma once
#include <cstdint>
#include <cstdio>
#define INPUT 1
#define ANALOG3 36
#define ANALOG4 39
static unsigned long fake_ms = 0;
static int fake_adc[40];
inline unsigned long millis() { return fake_ms; }
inline void delay(unsigned long ms) { fake_ms += ms; }
inline void pinMode(int, int) {}
inline int analogRead(int pin) { return fake_adc[pin]; }
#define PTH(a, b) std::printf("%s%g", a, (double)(b))
#define PTHL(a, b) std::printf("%s%g\n", a, (double)(b))
struct FakeQueue {
  int n = 0;
  int adds = 0;
  int size() { return n; }
  template <typename T>
  void addTask(char, T *, int = 0) { adds++; }
};
static FakeQueue fakeQueue;
static FakeQueue *tQueue = &fakeQueue;
