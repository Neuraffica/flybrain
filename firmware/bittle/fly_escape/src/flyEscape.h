/* flyEscape.h - OpenCatEsp32 extension module: the fly looming-escape circuit, on board.
 *
 * Reads the Petoi double IR distance sensor on ANALOG3 (GPIO 36, "left") and ANALOG4
 * (GPIO 39, "right") on every module poll, averages the readings, and every
 * FLY_ESCAPE_STEP_MS feeds the average to the exported model (fly_escape_all.h, generated
 * by `uv run flybrain-export escape --out ...`). On a trigger it queues a short backward walk
 * and a stand through the firmware's own task queue, at most once per
 * FLY_ESCAPE_MIN_INTERVAL_MS. Calibrates the empty-scene baseline at activation.
 *
 * Enable with the serial command "XE" (extension E), disable with "Xe" or "X".
 * "XEP" also turns the live telemetry line on, "XEp" turns it off (like the camera's XCP/XCp).
 *
 * Messages go to every port (USB, Bluetooth, Grove serial) through printToAllPorts(), one
 * line each, "@FE <what>;" (the ';' ends it even when a Bluetooth link drops the newline):
 *   @FE on tel=0;                 module enabled (tel=1: telemetry on); after X?, XE, Xe, X
 *   @FE off;                      module disabled
 *   @FE base L=3595 R=3580;       resting levels (ADC counts): at activation, and after each
 *                                 "@FE on" status once measured
 *   @FE trig side=-1 drive=1.05;  trigger, escape queued (side +1 left, -1 right, 0 ahead)
 *   @FE done;                     the escape's walk and stand are over, listening again
 *   @FE t 3590 3580 0.06 0.01;    telemetry, every FLY_ESCAPE_TEL_MS while on and at each
 *                                 trigger: raw left, raw right (step averages), giant-fibre
 *                                 activity left, right as a fraction of that side's threshold
 */
#ifndef FLY_ESCAPE_MODULE_H
#define FLY_ESCAPE_MODULE_H

#ifndef FLY_ESCAPE_MODEL
#define FLY_ESCAPE_MODEL 0  // 0 = FE_MODEL_CONNECTOME, 1 = FE_MODEL_LOOMING (baseline)
#endif
#if FLY_ESCAPE_MODEL == 1
#define FE_WITH_LOOMING 1  // the baseline is compiled only when selected
#endif
#include <stdarg.h>
#include <stdio.h>
#include "fly_escape_all.h"
#define FLY_ESCAPE_STEP_MS 35            // model step; 25-90 ms is the validated range
#define FLY_ESCAPE_MIN_INTERVAL_MS 3000  // between escapes
#define FLY_ESCAPE_BACK_MS 1000          // duration of the backward walk
#define FLY_ESCAPE_CAL_SAMPLES 64
#define FLY_ESCAPE_TEL_MS 200            // telemetry period (5 lines/s, about 150 B/s)
#define FLY_ESCAPE_PIN_LEFT ANALOG3      // GPIO 36 (prox_left in the laptop demo)
#define FLY_ESCAPE_PIN_RIGHT ANALOG4     // GPIO 39
#if FLY_ESCAPE_MODEL == 1                // what the telemetry divides gf_l, gf_r by (drive = 1)
#define FLY_ESCAPE_TH_L FE_LOOM_THRESHOLD
#define FLY_ESCAPE_TH_R FE_LOOM_THRESHOLD
#else
#define FLY_ESCAPE_TH_L FE_TH_L
#define FLY_ESCAPE_TH_R FE_TH_R
#endif

fe_state_t flyEscapeState;
uint32_t flyEscapeSumL = 0, flyEscapeSumR = 0;
uint16_t flyEscapeCount = 0;
unsigned long flyEscapeLastStep = 0;
unsigned long flyEscapeLastEscape = 0;
unsigned long flyEscapeLastTel = 0;
bool flyEscapeTelemetry = false;  // off at every activation: other Bluetooth clients stay quiet
bool flyEscapeBusy = false;       // an escape was queued and has not been reported done
bool flyEscapeCalibrated = false; // resting levels measured since the module was enabled

void flyEscapeSay(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void flyEscapeSay(const char *fmt, ...) {
  char buf[48];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  printToAllPorts(buf);
}

void flyEscapeSayBase() {
  flyEscapeSay("@FE base L=%d R=%d;", (int)(flyEscapeState.baseline_l + 0.5f),
               (int)(flyEscapeState.baseline_r + 0.5f));
}

void flyEscapeStatus(bool on) {  // called from showModuleStatus() (X?, and after XE/Xe/X)
  if (on) {
    flyEscapeSay("@FE on tel=%d;", flyEscapeTelemetry ? 1 : 0);
    if (flyEscapeCalibrated)  // so a listener that connects later also learns the levels
      flyEscapeSayBase();
  } else
    flyEscapeSay("@FE off;");
}

void flyEscapeOptions(const char *cmd, bool on) {  // "E..." of an "XE..." command
  bool said = false;
  for (const char *o = cmd + 1; *o && *o != '~'; o++)
    if (on && (*o == 'P' || *o == 'p')) {
      flyEscapeTelemetry = *o == 'P';
      said = true;
    }
  if (said)
    flyEscapeStatus(on);
}

void flyEscapeSetup() {
  uint16_t calL[FLY_ESCAPE_CAL_SAMPLES], calR[FLY_ESCAPE_CAL_SAMPLES];
  pinMode(FLY_ESCAPE_PIN_LEFT, INPUT);
  pinMode(FLY_ESCAPE_PIN_RIGHT, INPUT);
  fe_init(&flyEscapeState, FLY_ESCAPE_MODEL);
  // keep hands away during activation: the mean of these samples is the empty-scene level
  for (int i = 0; i < FLY_ESCAPE_CAL_SAMPLES; i++) {
    calL[i] = analogRead(FLY_ESCAPE_PIN_LEFT);
    calR[i] = analogRead(FLY_ESCAPE_PIN_RIGHT);
    delay(2);
  }
  fe_calibrate(&flyEscapeState, calL, calR, FLY_ESCAPE_CAL_SAMPLES);
  flyEscapeCalibrated = true;
  flyEscapeSayBase();
  flyEscapeSumL = flyEscapeSumR = 0;
  flyEscapeCount = 0;
  flyEscapeLastStep = millis();
  flyEscapeLastEscape = millis() - FLY_ESCAPE_MIN_INTERVAL_MS;
  flyEscapeTelemetry = flyEscapeBusy = false;
}

void flyEscapeStop() {
  fe_reset(&flyEscapeState);
  flyEscapeTelemetry = flyEscapeBusy = flyEscapeCalibrated = false;
}

void read_flyEscape() {
  unsigned long now = millis();
  if (flyEscapeBusy) {  // polled again only once the task queue is empty: the escape is over
    flyEscapeBusy = false;
    flyEscapeSay("@FE done;");
  }
  flyEscapeSumL += analogRead(FLY_ESCAPE_PIN_LEFT);
  flyEscapeSumR += analogRead(FLY_ESCAPE_PIN_RIGHT);
  flyEscapeCount++;
  unsigned long elapsed = now - flyEscapeLastStep;
  if (elapsed < FLY_ESCAPE_STEP_MS)
    return;
  float rawL = (float)flyEscapeSumL / flyEscapeCount;
  float rawR = (float)flyEscapeSumR / flyEscapeCount;
  flyEscapeSumL = flyEscapeSumR = 0;
  flyEscapeCount = 0;
  flyEscapeLastStep = now;
  // after a long pause (e.g. while a skill ran) restart the dynamics instead of feeding one
  // huge step
  if (elapsed > 500) {
    fe_reset(&flyEscapeState);
    elapsed = FLY_ESCAPE_STEP_MS;
  }
  fe_result_t r = fe_step(&flyEscapeState, rawL, rawR, (float)elapsed);
  // also at a trigger, so the 5 Hz trace shows the approach (it lasts only 70-250 ms)
  if (flyEscapeTelemetry && (now - flyEscapeLastTel >= FLY_ESCAPE_TEL_MS || r.trigger)) {
    flyEscapeLastTel = now;
    flyEscapeSay("@FE t %d %d %.2f %.2f;", (int)(rawL + 0.5f), (int)(rawR + 0.5f),
                 (double)(flyEscapeState.gf_l / FLY_ESCAPE_TH_L),
                 (double)(flyEscapeState.gf_r / FLY_ESCAPE_TH_R));
  }
  if (r.trigger && now - flyEscapeLastEscape >= FLY_ESCAPE_MIN_INTERVAL_MS && tQueue->size() == 0) {
    flyEscapeLastEscape = now;
    flyEscapeSay("@FE trig side=%d drive=%.2f;", (int)r.side, (double)r.drive);
    tQueue->addTask('k', "bk", FLY_ESCAPE_BACK_MS);
    tQueue->addTask('k', "up");
    flyEscapeBusy = true;
  }
}

#endif  // FLY_ESCAPE_MODULE_H
