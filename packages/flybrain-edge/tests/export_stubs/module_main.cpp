// Host check of firmware/bittle/fly_escape/src/flyEscape.h against stubs: a hand comes in
// on the left channel after calibration; the module must queue the escape exactly once and
// print its "@FE ...;" lines (status, resting level, telemetry while on, trigger, done).
#include "opencat_stub.h"
#include "flyEscape.h"

static int count(const char *needle) {
  int n = 0;
  for (size_t i = fake_out.find(needle); i != std::string::npos; i = fake_out.find(needle, i + 1))
    n++;
  return n;
}

int main() {
  fake_adc[36] = fake_adc[39] = 3400;
  flyEscapeSetup();
  flyEscapeStatus(true);
  flyEscapeOptions("EP~", true);  // "XEP": telemetry on
  int busy_ms = 0;
  for (int ms = 0; ms < 3000; ms++) {  // 1 kHz polling
    fake_ms++;
    int t = ms - 1000;                  // hand reaches the left channel at 1 s, over 120 ms
    fake_adc[36] = t < 0 ? 3400 : (t > 120 ? 0 : 3400 - 3400 * t / 120);
    if (fakeQueue.n) {                  // the firmware polls modules only with an empty queue
      if (++busy_ms >= 1200)
        fakeQueue.n = busy_ms = 0;
      continue;
    }
    read_flyEscape();
  }
  flyEscapeOptions("Ep~", true);  // "XEp": telemetry off
  bool telemetry_after_p = flyEscapeTelemetry;
  flyEscapeOptions("EP~", true);
  flyEscapeStop();  // "Xe" must also switch it off
  bool ok = fakeQueue.adds == 2                        // "bk" then "up"
            // once at activation, then after every "on" status
            && count("@FE base L=3400 R=3400;\n") == 1 + count("@FE on tel=")  //
            && count("@FE on tel=0;\n") == 2 && count("@FE on tel=1;\n") == 2  //
            && count("@FE trig side=1 drive=") == 1 && count("@FE done;\n") == 1  //
            && count("@FE t ") >= 7 && count("@FE t ") <= 12  // 5/s for 1.8 s, + one at the trigger
            && !telemetry_after_p && !flyEscapeTelemetry;
  std::printf("\nadds=%d telemetry=%d ok=%d\n", fakeQueue.adds, count("@FE t "), ok);
  return ok ? 0 : 1;
}
