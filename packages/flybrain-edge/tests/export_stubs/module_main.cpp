// Host check of firmware/bittle/fly_escape/src/flyEscape.h against stubs: a hand comes in
// on the left channel after calibration; the module must queue the escape exactly once.
#include "opencat_stub.h"
#include "flyEscape.h"

int main() {
  fake_adc[36] = fake_adc[39] = 3400;
  flyEscapeSetup();
  for (int ms = 0; ms < 3000; ms++) {  // 1 kHz polling
    fake_ms++;
    int t = ms - 1000;                  // hand reaches the left channel at 1 s, over 120 ms
    fake_adc[36] = t < 0 ? 3400 : (t > 120 ? 0 : 3400 - 3400 * t / 120);
    read_flyEscape();
  }
  std::printf("\nadds=%d\n", fakeQueue.adds);
  return fakeQueue.adds == 2 ? 0 : 1;  // "bk" then "up"
}
