/* flyPursuit.h - OpenCatEsp32 extension module P: the Petoi AI Vision camera read in its own
 * task, and (optionally) the fly pursuit circuit turning the head on the robot itself.
 *
 * The circuit: 264 of the 265 neurons of the "pursuit" circuit from the fly connectome (one
 * LC10a has no path to DNa02 and is left out; this cannot change the output), exported by
 * `uv run flybrain-export pursuit --single-header --out firmware/bittle/fly_pursuit/src`.
 *
 * Commands (the firmware's X command; capital letters, so end each with '~'):
 *   XP~   module on, passive: camera lines only, the robot does not move
 *   Xp~   module off (also stops the camera's continuous detection)
 *   XPP~  result lines on (default)      XPp~  result lines off
 *   XPB~  on-board control on: the robot sits, then the head follows the face
 *   XPb~  on-board control off (the head stays where it is)
 * Host lines (all ports, one per line, each ends with ';'): see README.md, "Host lines".
 *
 * Not run on hardware yet. Requirements and how each is met: README.md.
 */
#ifndef FLY_PURSUIT_MODULE_H
#define FLY_PURSUIT_MODULE_H

#include <stdarg.h>
#include <Wire.h>
#include "esp32-hal-i2c.h"
#include "flyPursuitCore.h"

/* ---- I2C and the camera's SSCMA transport (from Seeed_Arduino_SSCMA, read 2026-11-18) ---- */
#define FP_ADDR 0x62              // MEASURED: the camera's SSCMA side answers here
#define FP_ADDR_OTHER 0x28        // MEASURED: also on the camera module (with 0x79); present
                                  // while 0x62 is missing = the camera's AI side is stuck
#define FP_FEATURE 0x10           // SSCMA transport feature code (library source)
#define FP_CMD_READ 0x01          // (library source)
#define FP_CMD_WRITE 0x02         // (library source)
#define FP_CMD_AVAIL 0x03         // (library source)
#define FP_CHUNK 64               // CHOSEN: bytes per read transaction. Must stay below Wire's
                                  // 128-byte buffer (the library asks for 250 at once)
#define FP_CMD_GAP_MS 2           // from the library: wait between a request and its read
#define FP_POLL_MS 10             // CHOSEN: ask the camera for new bytes every 10 ms
#define FP_LOCK_WAIT_MS 20        // CHOSEN: wait at most this long for the IMU to free the bus
#define FP_REPLY_MS 1500          // CHOSEN: wait for a command's reply (library: 1000-3000 ms)
#define FP_STALL_MS 1500          // CHOSEN: no result for this long while running = stalled
                                  // (MEASURED rate 15.4/s, longest gap 90 ms on its USB port)
#define FP_FAILS_TO_RECOVER 5     // CHOSEN: I2C failures in a row before a bus recovery
#define FP_RETRY_MS 2000          // CHOSEN: wait before trying to start the camera again
#define FP_STACK_BYTES 4096       // CHOSEN: the camera task's stack (static, never freed)
#define FP_TASK_PRIORITY 1        // CHOSEN: same as the IMU task
#define FP_TASK_CORE 0            // CHOSEN: same core as the IMU task and the stock camera task
#define FP_SENSOR_OPT_ANGLE_90 (1 << 12)  // the stock firmware's setting for a Bittle X
#define FP_LINE_LEN 96

/* error kinds, printed as "@FP err <name> <count>;" */
enum {
  FP_E_I2C = 0,  // an I2C transaction failed (NACK, timeout, short read)
  FP_E_LOCK,     // the IMU did not free the bus within FP_LOCK_WAIT_MS
  FP_E_STALL,    // running, but no result for FP_STALL_MS
  FP_E_OVERFLOW, // a reply did not fit in the buffer and was dropped
  FP_E_CMD,      // an AT command got no reply or an error code
  FP_E_NOCAM,    // nothing answers at 0x62 or 0x28: no camera on the I2C socket
  FP_E_SILENT,   // 0x28 answers, 0x62 does not: the camera's AI side is stuck (power-cycle it)
  FP_E_RECOVER,  // a bus recovery was made (count = recoveries since power-on)
  FP_E_BUSHELD,  // SDA or SCL still low after a recovery
  FP_E_COUNT
};
static const char *const FP_E_NAME[FP_E_COUNT] = {"i2c",    "lock",  "stall",   "overflow", "cmd",
                                                  "nocam",  "silent", "recover", "busheld"};

/* What the camera task publishes for the main loop (copied under fpMux). */
struct FpShared {
  uint16_t seq;           // camera results since power-on (wraps)
  FpMsg last;             // the latest result
  uint32_t t_meas_ms;     // estimated time the picture was taken
  uint32_t err[FP_E_COUNT];
  uint16_t starts;        // continuous detection started (since power-on)
  uint16_t stops;         // stopped with AT+BREAK
  uint8_t running;        // continuous detection is on
};

static portMUX_TYPE fpMux = portMUX_INITIALIZER_UNLOCKED;
static FpShared fpShared;          // written by the camera task, inside fpMux
static StaticTask_t fpTaskTcb;     // the task is created once, statically, never deleted
static StackType_t fpTaskStack[FP_STACK_BYTES];
static TaskHandle_t fpTask = NULL;
static volatile bool fpWantOn = false;  // set by the main loop, read by the task
static FpParser fpParser;               // the task's only buffer
static uint8_t fpIo[FP_CHUNK + 8];      // one transaction's bytes

/* main-loop state */
static FpShared fpSeen;                 // what the main loop has already reported
static bool fpLinesQ = true;            // XPP / XPp
static bool fpControlQ = false;         // XPB / XPb; never saved, off after every start
static FpControl fpCtl;

void fpSay(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void fpSay(const char *fmt, ...) {
  char buf[FP_LINE_LEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  printToAllPorts(buf);
}

/* =================================================================== camera task: the bus */
static void fpCount(uint8_t kind) {
  portENTER_CRITICAL(&fpMux);
  fpShared.err[kind]++;
  portEXIT_CRITICAL(&fpMux);
}

/* The firmware's convention (src/imu.h): a user of the shared bus raises its own flag and waits
 * for the others' flags to drop; the IMU task does not start a read while cameraLockI2c is up
 * (it waits up to 500 ms, then skips that read). The flag is raised FIRST and the others are
 * checked one tick later, so an IMU read that had already passed its own check has set
 * imuLockI2c by then and is waited for. No busy loop: every wait is vTaskDelay(1). */
static bool fpLock(uint32_t wait_ms = FP_LOCK_WAIT_MS) {
  cameraLockI2c = true;
  vTaskDelay(1);
  uint32_t t0 = millis();
  while (imuLockI2c || gestureLockI2c || eepromLockI2c) {
    if (millis() - t0 > wait_ms) {
      cameraLockI2c = false;
      fpCount(FP_E_LOCK);
      return false;
    }
    vTaskDelay(1);
  }
  return true;
}

static void fpUnlock() { cameraLockI2c = false; }

/* One SSCMA transport command (+ payload) to 0x62. Bounded by Wire's timeout (50 ms).
 * Byte layout from Seeed_Arduino_SSCMA: 0x10, cmd, len high, len low, payload or checksum. */
static bool fpTxLocked(uint8_t cmd, const uint8_t *data, uint16_t len) {
  Wire.beginTransmission(FP_ADDR);
  Wire.write(FP_FEATURE);
  Wire.write(cmd);
  Wire.write((uint8_t)(len >> 8));
  Wire.write((uint8_t)(len & 0xFF));
  if (data && len)
    Wire.write(data, len);  // a WRITE: no checksum bytes, exactly as the library sends one
  else {                    // READ and AVAIL: two zero checksum bytes, as the library sends
    Wire.write((uint8_t)0);
    Wire.write((uint8_t)0);
  }
  return Wire.endTransmission() == 0;
}

/* Request n bytes of kind cmd (READ or AVAIL) and read them into fpIo. */
static bool fpRequest(uint8_t cmd, uint16_t n) {
  if (!fpLock()) return false;
  bool ok = fpTxLocked(cmd, NULL, cmd == FP_CMD_READ ? n : 0);
  if (ok) {
    vTaskDelay(pdMS_TO_TICKS(FP_CMD_GAP_MS));
    ok = Wire.requestFrom((uint16_t)FP_ADDR, (size_t)n, true) == n;
    for (uint16_t i = 0; ok && i < n; i++) fpIo[i] = (uint8_t)Wire.read();
  }
  fpUnlock();
  if (!ok) fpCount(FP_E_I2C);
  return ok;
}

/* Bytes the camera has ready, or -1 on an I2C failure. */
static int32_t fpAvailable() {
  if (!fpRequest(FP_CMD_AVAIL, 2)) return -1;
  int32_t n = ((int32_t)fpIo[0] << 8) | fpIo[1];
  return n;
}

static bool fpWriteText(const char *text) {
  uint16_t len = (uint16_t)strlen(text);
  if (!fpLock()) return false;
  bool ok = fpTxLocked(FP_CMD_WRITE, (const uint8_t *)text, len);  // commands are < 40 bytes
  fpUnlock();
  if (!ok) fpCount(FP_E_I2C);
  return ok;
}

static bool fpProbe(uint8_t addr) {
  if (!fpLock()) return false;
  Wire.beginTransmission(addr);
  bool ok = Wire.endTransmission() == 0;
  fpUnlock();
  return ok;
}

static uint8_t fpFailsInRow = 0;

/* Pull what the camera has (at most one buffer's worth) into the parser and hand every frame
 * to fpOnMsg. Returns false on an I2C failure. */
static void fpPublish(const FpMsg &m, uint32_t t_avail);
static bool fpPump(int (*onResponse)(const FpMsg &, const char *), const char *want, int *code) {
  uint32_t t_avail = millis();
  int32_t n = fpAvailable();
  if (n < 0) return false;
  if (n > 0x4000) {  // nonsense length: count it as a failed transaction
    fpCount(FP_E_I2C);
    return false;
  }
  while (n > 0) {
    uint16_t room = fpParserRoom(&fpParser);
    uint16_t k = (uint16_t)(n < FP_CHUNK ? n : FP_CHUNK);
    if (k > room) k = room;
    if (!fpRequest(FP_CMD_READ, k)) return false;
    memcpy(fpWriteSpot(&fpParser), fpIo, k);
    fpParserAdd(&fpParser, k);
    n -= k;
    FpMsg m;
    while (fpParserNext(&fpParser, &m)) {
      if (m.kind == FP_MSG_RESULT)
        fpPublish(m, t_avail);
      else if (m.kind == FP_MSG_RESPONSE && want && code && onResponse && onResponse(m, want))
        *code = m.code;
    }
  }
  return true;
}

static int fpIsReplyTo(const FpMsg &m, const char *want) { return strcmp(m.name, want) == 0; }

/* Send "AT+<cmd>\r\n" and wait (polling, FP_POLL_MS apart) for the response named `name`.
 * Returns its code, or -1 if none arrived within FP_REPLY_MS. */
static int fpAt(const char *cmd, const char *name) {
  char line[48];
  snprintf(line, sizeof(line), "AT+%s\r\n", cmd);
  if (!fpWriteText(line)) return -1;
  int code = -1;
  uint32_t t0 = millis();
  while (code < 0 && millis() - t0 < FP_REPLY_MS) {
    vTaskDelay(pdMS_TO_TICKS(FP_POLL_MS));
    fpPump(fpIsReplyTo, name, &code);
  }
  return code;
}

/* Bus recovery: SCL clocked 9 times as a GPIO with SDA released, then a STOP, then the I2C
 * driver re-installed on the firmware's pins at the clock it had. Done with the camera's flag
 * up, so the IMU task stays off the bus (it skips reads while the flag is up). The driver is
 * re-installed through the core's HAL (i2cDeinit / i2cInit), which is what Wire.begin() does
 * without re-allocating Wire's buffers; the ESP-IDF driver itself allocates its small control
 * block again when it is installed. */
static void fpRecoverBus() {
  fpLock(200);  // proceed even if the IMU never lets go: the bus is broken anyway
  cameraLockI2c = true;
  uint32_t hz = Wire.getClock();
  if (hz == 0) hz = 100000;
  i2cDeinit(0);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }
  pinMode(SDA, OUTPUT_OPEN_DRAIN);  // STOP: SDA low -> high while SCL is high
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  bool held = digitalRead(SDA) == LOW || digitalRead(SCL) == LOW;
  i2cInit(0, SDA, SCL, hz);
  fpUnlock();
  fpCount(FP_E_RECOVER);
  if (held) fpCount(FP_E_BUSHELD);
}

/* Start continuous detection. Returns true when the camera confirmed AT+INVOKE. */
static bool fpStartCamera() {
  fpParserReset(&fpParser);
  if (!fpProbe(FP_ADDR)) {
    fpCount(fpProbe(FP_ADDR_OTHER) ? FP_E_SILENT : FP_E_NOCAM);
    return false;
  }
  fpAt("BREAK", "BREAK");  // a detection left running from before an ESP32 restart
  char cmd[32];
  uint16_t opt = !strcmp(MODEL, "Bittle X+Arm") ? 0 : FP_SENSOR_OPT_ANGLE_90;  // as stock
  snprintf(cmd, sizeof(cmd), "SENSOR=1,1,%u", (unsigned)opt);  // 240 x 240, rotation
  if (fpAt(cmd, "SENSOR") != 0) {
    fpCount(FP_E_CMD);
    return false;
  }
  if (fpAt("INVOKE=-1,0,1", "INVOKE") != 0) {  // forever, no filter, results only
    fpCount(FP_E_CMD);
    return false;
  }
  portENTER_CRITICAL(&fpMux);
  fpShared.starts++;
  fpShared.running = 1;
  portEXIT_CRITICAL(&fpMux);
  return true;
}

static void fpStopCamera() {
  fpAt("BREAK", "BREAK");  // best effort; the result is not needed
  fpParserReset(&fpParser);
  portENTER_CRITICAL(&fpMux);
  fpShared.running = 0;
  fpShared.stops++;
  portEXIT_CRITICAL(&fpMux);
}

static uint32_t fpLastResultMs = 0;

static void fpPublish(const FpMsg &m, uint32_t t_avail) {
  fpLastResultMs = millis();
  portENTER_CRITICAL(&fpMux);
  fpShared.seq++;
  fpShared.last = m;
  fpShared.t_meas_ms = t_avail - m.perf_ms;  // ASSUMED: the picture was taken just before
                                             // the camera's own preprocessing began
  portEXIT_CRITICAL(&fpMux);
}

static void fpTaskMain(void *) {
  bool running = false;
  uint32_t nextTry = 0;
  for (;;) {
    if (!fpWantOn) {
      if (running) {
        fpStopCamera();
        running = false;
      }
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));  // idle until enabled
      continue;
    }
    if (!running) {
      if ((int32_t)(millis() - nextTry) < 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        continue;
      }
      running = fpStartCamera();
      fpFailsInRow = 0;
      fpLastResultMs = millis();
      if (!running) nextTry = millis() + FP_RETRY_MS;
      continue;
    }
    if (fpPump(NULL, NULL, NULL))
      fpFailsInRow = 0;
    else if (++fpFailsInRow >= FP_FAILS_TO_RECOVER) {
      fpRecoverBus();
      fpFailsInRow = 0;
      running = false;  // start the camera again on the next pass
      portENTER_CRITICAL(&fpMux);
      fpShared.running = 0;
      portEXIT_CRITICAL(&fpMux);
      continue;
    }
    if (millis() - fpLastResultMs > FP_STALL_MS) {
      fpCount(FP_E_STALL);
      running = false;  // send AT+INVOKE again
      portENTER_CRITICAL(&fpMux);
      fpShared.running = 0;
      portEXIT_CRITICAL(&fpMux);
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(FP_POLL_MS));
  }
}

/* ==================================================================== main loop side */
void flyPursuitStatus(bool on) {  // after the module list (X?, and after every X change)
  if (on)
    fpSay("@FP on l=%d b=%d;", fpLinesQ ? 1 : 0, fpControlQ ? 1 : 0);
  else
    fpSay("@FP off;");
}

static void fpMoveHead(float pan_deg) {
  /* The same call the firmware's "i 0 <angle>" ends in: after a posture it sets targetHead[0]
   * and manualHeadQ and the posture's perform() moves joint 0 with calibratedPWM(); here the
   * target is set the same way and calibratedPWM() is called at once, so the head moves even
   * when no skill is being performed. calibratedPWM() clamps to the joint's angle limits. */
  int a = (int)lroundf(FP_PAN_SIGN * pan_deg);
  targetHead[0] = a;
  manualHeadQ = true;
  calibratedPWM(0, a);
}

void flyPursuitSetup() {  // initModule(): XP~, and at boot when saved as active
  if (fpTask == NULL) {
    fpParserReset(&fpParser);
    fpTask = xTaskCreateStaticPinnedToCore(fpTaskMain, "TaskFlyPursuit", FP_STACK_BYTES, NULL,
                                           FP_TASK_PRIORITY, fpTaskStack, &fpTaskTcb, FP_TASK_CORE);
  }
  fpControlQ = false;  // passive: the robot does not move until XPB~
  fpLinesQ = true;
  fpWantOn = true;
  if (fpTask) xTaskNotifyGive(fpTask);
}

void flyPursuitStop() {  // stopModule(): Xp~, X~, or another module enabled
  fpControlQ = false;
  fpWantOn = false;
  if (fpTask) xTaskNotifyGive(fpTask);
}

void flyPursuitOptions(const char *cmd, bool on) {  // "P..." of an "XP..." command
  bool said = false;
  for (const char *o = cmd + 1; *o && *o != '~'; o++) {
    if (!on) break;
    if (*o == 'P' || *o == 'p') {
      fpLinesQ = *o == 'P';
      said = true;
    } else if (*o == 'B' && !fpControlQ) {
      loadBySkillName("sit");  // as the stock camera module does when it starts
      fpControlReset(&fpCtl, millis(), FP_PAN_SIGN * (float)currentAng[0]);
      fpMoveHead(fpCtl.pan_deg);
      fpControlQ = true;
      said = true;
    } else if (*o == 'b') {
      fpControlQ = false;
      said = true;
    }
  }
  if (said) flyPursuitStatus(on);
}

/* Called on every pass of the main loop (readSignal), whether or not the robot is idle. */
void read_flyPursuit() {
  FpShared s;
  portENTER_CRITICAL(&fpMux);
  s = fpShared;
  portEXIT_CRITICAL(&fpMux);
  for (uint8_t k = 0; k < FP_E_COUNT; k++)
    if (s.err[k] != fpSeen.err[k]) fpSay("@FP err %s %lu;", FP_E_NAME[k], (unsigned long)s.err[k]);
  if (s.starts != fpSeen.starts) fpSay("@FP cam run %u;", (unsigned)s.starts);
  if (s.stops != fpSeen.stops) fpSay("@FP cam stop;");
  if (s.seq != fpSeen.seq && fpWantOn) {
    char line[FP_LINE_LEN];
    if (fpControlQ) {
      uint32_t now = millis();
      fpMoveHead(fpControlStep(&fpCtl, s.last.has_box, s.last.x, s.t_meas_ms, now));
    }
    if (fpLinesQ) {
      fpFormatResult(line, sizeof(line), s.seq, &s.last, fpControlQ ? &fpCtl : NULL);
      printToAllPorts(line);
    }
  }
  fpSeen = s;
}

/* Called once in initRobot() before Wire.begin(): if the camera was left holding SDA low
 * (seen once: the firmware then stopped in its boot I2C scan until the power was cycled),
 * clock SCL 9 times and send a STOP. Prints to USB only (Bluetooth is not up yet), and only
 * when a line was low. Nothing else changes. */
void flyPursuitBootBusCheck() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(20);
  int sda = digitalRead(SDA), scl = digitalRead(SCL);
  if (sda == HIGH && scl == HIGH) return;
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(20);
  char buf[64];
  snprintf(buf, sizeof(buf), "@FP boot bus sda=%d scl=%d now sda=%d scl=%d;", sda, scl,
           digitalRead(SDA), digitalRead(SCL));
  Serial.println(buf);
}

#endif  // FLY_PURSUIT_MODULE_H
