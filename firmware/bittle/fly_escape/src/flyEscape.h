/* flyEscape.h - OpenCatEsp32 extension module: the fly looming-escape circuit, on board.
 *
 * DRAFT. Not compiled for the ESP32 and not run on a robot (see ../README.md).
 *
 * Reads the Petoi double IR distance sensor on ANALOG3 (GPIO 36, "left") and ANALOG4
 * (GPIO 39, "right") on every module poll, averages the readings, and every
 * FLY_ESCAPE_STEP_MS feeds the average to the exported model (fly_escape_all.h, generated
 * by `uv run flybrain-export escape --out ...`). On a trigger it queues a short backward walk
 * and a stand through the firmware's own task queue, at most once per
 * FLY_ESCAPE_MIN_INTERVAL_MS. Calibrates the empty-scene baseline at activation.
 *
 * Enable with the serial command "XE" (extension E), disable with "Xe" or "X".
 */
#ifndef FLY_ESCAPE_MODULE_H
#define FLY_ESCAPE_MODULE_H

#ifndef FLY_ESCAPE_MODEL
#define FLY_ESCAPE_MODEL 0  // 0 = FE_MODEL_CONNECTOME, 1 = FE_MODEL_LOOMING (baseline)
#endif
#if FLY_ESCAPE_MODEL == 1
#define FE_WITH_LOOMING 1  // the baseline is compiled only when selected
#endif
#include "fly_escape_all.h"
#define FLY_ESCAPE_STEP_MS 35            // model step; 25-90 ms is the validated range
#define FLY_ESCAPE_MIN_INTERVAL_MS 3000  // between escapes
#define FLY_ESCAPE_BACK_MS 1000          // duration of the backward walk
#define FLY_ESCAPE_CAL_SAMPLES 64
#define FLY_ESCAPE_PIN_LEFT ANALOG3      // GPIO 36 (prox_left in the laptop demo)
#define FLY_ESCAPE_PIN_RIGHT ANALOG4     // GPIO 39

fe_state_t flyEscapeState;
uint32_t flyEscapeSumL = 0, flyEscapeSumR = 0;
uint16_t flyEscapeCount = 0;
unsigned long flyEscapeLastStep = 0;
unsigned long flyEscapeLastEscape = 0;

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
  PTH("flyEscape baseline L", flyEscapeState.baseline_l);
  PTHL(" R", flyEscapeState.baseline_r);
  flyEscapeSumL = flyEscapeSumR = 0;
  flyEscapeCount = 0;
  flyEscapeLastStep = millis();
  flyEscapeLastEscape = millis() - FLY_ESCAPE_MIN_INTERVAL_MS;
}

void flyEscapeStop() {
  fe_reset(&flyEscapeState);
}

void read_flyEscape() {
  unsigned long now = millis();
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
  if (r.trigger && now - flyEscapeLastEscape >= FLY_ESCAPE_MIN_INTERVAL_MS && tQueue->size() == 0) {
    flyEscapeLastEscape = now;
    PTH("flyEscape trigger, side ", r.side);
    PTHL(" drive ", r.drive);
    tQueue->addTask('k', "bk", FLY_ESCAPE_BACK_MS);
    tQueue->addTask('k', "up");
  }
}

#endif  // FLY_ESCAPE_MODULE_H
