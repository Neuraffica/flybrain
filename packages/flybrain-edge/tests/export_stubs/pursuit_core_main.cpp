// Host check of firmware/bittle/fly_pursuit/src/flyPursuitCore.h: the SSCMA reply parser (fed
// in uneven pieces, with NUL filler, a log frame, an oversized reply and empty results), the
// head control step (direction, rate limit, pan limit, hold after 3 s, efference copy) and the
// "@FP" line format, and the "follow after power-on" decision (fpAutoDue). Prints the failed checks; exit code 0 when all pass.
#include <cstdio>
#include <string>

#include "flyPursuitCore.h"

static int failures = 0;
#define CHECK(c)                                                   \
  do {                                                             \
    if (!(c)) {                                                    \
      std::printf("FAIL line %d: %s\n", __LINE__, #c);             \
      failures++;                                                  \
    }                                                              \
  } while (0)

static FpParser P;

static int feedAll(const std::string &s, FpMsg *out, int max, unsigned seed) {
  int n = 0;
  size_t i = 0;
  while (i < s.size()) {
    seed = seed * 1103515245u + 12345u;
    size_t k = 1 + (seed >> 16) % 64;  // pieces of 1..64 bytes, like the I2C reads
    if (i + k > s.size()) k = s.size() - i;
    fpParserFeed(&P, s.data() + i, (uint16_t)k);
    i += k;
    FpMsg m;
    while (fpParserNext(&P, &m))
      if (n < max) out[n++] = m;
  }
  return n;
}

int main() {
  // ---- parser -------------------------------------------------------------------------
  fpParserReset(&P);
  std::string big(1500, 'x');
  std::string s;
  s += std::string("\r{\"type\": 0, \"name\": \"INVOKE\", \"code\": 0, \"data\": {\"model\": {\"id\": 1, "
                   "\"type\": 3, \"address\": 4194304, \"size\": 0}, \"algorithm\": {\"type\": 3, "
                   "\"categroy\": 1, \"input_from\": 1, \"config\": {\"tscore\": 50, \"tiou\": 45}}, "
                   "\"sensor\": {\"id\": 1, \"type\": 1, \"state\": 1, \"opt_id\": 0, "
                   "\"opt_detail\": \"240x240 Auto\"}}}\n");
  s += "\r{\"type\": 1, \"name\": \"INVOKE\", \"code\": 0, \"data\": {\"count\": 1, \"perf\": [7, 20, 1], "
       "\"boxes\": [[150, 130, 83, 90, 72, 0], [40, 100, 30, 30, 81, 0]]}}\n";
  s += std::string(5, '\0');  // filler bytes
  s += "\r{\"type\": 2, \"name\": \"LOG\", \"code\": 0, \"data\": \"anything } here\"}\n";
  s += "\r{\"type\": 1, \"name\": \"INVOKE\", \"code\": 0, \"data\": {\"count\": 2, \"perf\": [6, 21, 0], "
       "\"boxes\": []}}\n";
  s += "\r{\"type\": 0, \"name\": \"" + big + "\"}\n";  // longer than the buffer
  s += "\r{\"type\":1,\"name\":\"INVOKE\",\"code\":0,\"data\":{\"perf\":[1,2,3],\"boxes\":[[120,120,10,"
       "12,55,0]]}}\n";
  s += "\r{\"type\": 0, \"name\": \"BREAK\", \"code\": 0, \"data\": {}}\n";
  FpMsg got[16];
  int n = feedAll(s, got, 16, 7u);
  CHECK(n == 6);
  CHECK(got[0].kind == FP_MSG_RESPONSE && !strcmp(got[0].name, "INVOKE") && got[0].code == 0);
  CHECK(got[1].kind == FP_MSG_RESULT && got[1].has_box);
  CHECK(got[1].score == 81 && got[1].x == 40 - 120 && got[1].y == 100 - 120);  // best score
  CHECK(got[1].w == 30 && got[1].h == 30 && got[1].perf_ms == 28);
  CHECK(got[2].kind == FP_MSG_OTHER && !strcmp(got[2].name, "LOG"));
  CHECK(got[3].kind == FP_MSG_RESULT && !got[3].has_box && got[3].perf_ms == 27);
  CHECK(got[4].kind == FP_MSG_RESULT && got[4].has_box && got[4].x == 0 && got[4].h == 12);
  CHECK(got[5].kind == FP_MSG_RESPONSE && !strcmp(got[5].name, "BREAK"));
  CHECK(P.overflows >= 1);
  CHECK(P.len < 4);  // nothing left over but at most a partial frame start

  // ---- control --------------------------------------------------------------------------
  static FpControl c;
  uint32_t t = 1000;
  fpControlReset(&c, t, 0.0f);
  CHECK(fpBearingDeg(-60) > 14.9f && fpBearingDeg(-60) < 15.1f);  // dog's left = + bearing
  float prev = c.pan_deg, max_step = 0.0f;
  for (int i = 0; i < 150; i++) {  // a face held 15 deg to the left, 15 results a second
    t += 67;
    float pan = fpControlStep(&c, 1, -60, t - 30, t);
    float step = pan - prev;
    if (step > max_step) max_step = step;
    CHECK(step >= -1e-4f);  // only ever turns left
    prev = pan;
  }
  CHECK(max_step <= 60.0f * 0.067f + 1e-3f);    // 60 deg/s rate limit
  CHECK(c.pan_deg <= 50.0f && c.pan_deg > 45.0f);  // ran into the pan limit (face stays left)
  char line[96];
  FpMsg face = got[1];
  fpFormatResult(line, sizeof(line), 42, &face, NULL);
  CHECK(!strcmp(line, "@FP 42 -80 -20 30 30 81;"));
  fpFormatResult(line, sizeof(line), 7, &got[3], NULL);
  CHECK(!strcmp(line, "@FP 7 -;"));
  fpFormatResult(line, sizeof(line), 43, &face, &c);
  CHECK(!strncmp(line, "@FP 43 -80 -20 30 30 81 b=", 26) && strstr(line, " t=") &&
        strstr(line, " p=50.0;"));

  // efference copy: the face is where it was, the head has turned 10 deg left since the
  // picture -> the corrected bearing is 10 deg less
  fpControlReset(&c, 0, 0.0f);
  fpPanRecord(&c, 100, 0.0f);
  c.pan_deg = 10.0f;
  fpPanRecord(&c, 200, 10.0f);
  fpControlStep(&c, 1, -60, 150, 210);  // picture at 150 ms, pan was 0 then
  CHECK(c.bearing_deg > 4.9f && c.bearing_deg < 5.1f);

  // no face: the head holds after 3 s and stays where it is
  fpControlReset(&c, 0, 20.0f);
  t = 0;
  float at_hold = 0.0f;
  int holds = 0;
  for (int i = 0; i < 90; i++) {  // 6 s of empty results
    t += 67;
    float pan = fpControlStep(&c, 0, 0, t, t);
    if (c.holding) {
      if (!holds) at_hold = pan;
      holds++;
      CHECK(pan == at_hold && c.cmd_deg_s == 0.0f);
    }
    CHECK(!c.holding || t >= FP_HOLD_AFTER_MS);
  }
  CHECK(holds > 30);
  fpFormatResult(line, sizeof(line), 9, &got[3], &c);
  CHECK(strstr(line, "@FP 9 - t=0.0 p=") == line && strstr(line, " hold;"));

  // ---- follow after power-on: due only when saved, not done, queue empty and 3 s passed ----
  CHECK(FP_AUTO_DELAY_MS == 3000);
  CHECK(fpAutoDue(1, 0, 3000, 1) == 1);
  CHECK(fpAutoDue(1, 0, 60000, 1) == 1);
  CHECK(fpAutoDue(1, 0, 2999, 1) == 0);  // too early
  CHECK(fpAutoDue(1, 0, 0, 1) == 0);
  CHECK(fpAutoDue(0, 0, 5000, 1) == 0);  // not saved
  CHECK(fpAutoDue(1, 1, 5000, 1) == 0);  // fired or cancelled already
  CHECK(fpAutoDue(1, 0, 5000, 0) == 0);  // boot task still queued
  int fired = 0;
  for (uint32_t ms = 0; ms <= 10000; ms += 10) {  // one-shot as flyPursuit.h uses it
    if (fpAutoDue(1, fired > 0, ms, ms >= 4000)) fired++;
  }
  CHECK(fired == 1);

  std::printf("results=%d overflows=%u failures=%d\n", n, (unsigned)P.overflows, failures);
  return failures ? 1 : 0;
}
