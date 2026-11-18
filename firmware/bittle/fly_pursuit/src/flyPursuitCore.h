/* flyPursuitCore.h - the parts of the fly_pursuit module that do not touch the hardware:
 * the SSCMA reply parser, the on-board head control step and the host line formatting.
 * Plain C++ with no Arduino or FreeRTOS calls, so the same file is compiled and tested on the
 * laptop (packages/flybrain-edge/tests/test_onboard_pursuit.py). flyPursuit.h includes it.
 *
 * Every constant says whether it is MEASURED (on this robot, see
 * packages/flybrain-edge/NOTES.md), ASSUMED (not measured) or CHOSEN (a design decision).
 */
#ifndef FLY_PURSUIT_CORE_H
#define FLY_PURSUIT_CORE_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fly_pursuit_all.h"

/* ---- camera (Petoi AI Vision = Seeed Grove Vision AI V2, SSCMA firmware) ------------------ */
#define FP_FRAME_PX 240        // MEASURED: the camera's image is 240 x 240 pixels
#define FP_FOV_DEG 60.0f       // ASSUMED: field of view across the image (one head step gave
                               // about 57 deg; not properly measured); same as the laptop demo
#define FP_RX_BUF 1024         // CHOSEN: reply buffer. An INVOKE event with one box is about
                               // 150-250 bytes; the INVOKE response with model info is larger
#define FP_SCORE_MIN 0         // CHOSEN: no extra score filter (the camera's model already
                               // uses its own threshold, MEASURED 50 on this camera)

/* ---- on-board head control (mirrors demos/fly-pursuit/src/fly_pursuit/robot.py) ---------- */
#define FP_HEAD_RATE_DEG_S 60.0f  // CHOSEN (as the laptop demo): the head turns at most this fast
#define FP_PAN_LIMIT_DEG 50.0f    // CHOSEN (as the laptop demo): pan target kept within +-50 deg
#define FP_HOLD_AFTER_MS 3000     // CHOSEN (as the laptop demo): no face for 3 s -> head holds
#define FP_DT_MIN_S 0.001f        // CHOSEN: shortest step given to the circuit
#define FP_DT_MAX_S 0.5f          // CHOSEN: longest step given to the circuit
#define FP_RESET_GAP_MS 1000      // CHOSEN: no camera result for this long -> circuit reset
#define FP_PAN_DT_MAX_S 0.15f     // CHOSEN: the pan target moves by at most rate x 0.15 s per
                                  // result (9 deg), so a late result does not make a jump
#define FP_AROUSAL 1.0f           // CHOSEN: the circuit's default arousal
#define FP_PAN_HISTORY 32         // CHOSEN: pan targets remembered (about 2 s at 15 Hz)
#define FP_PAN_SIGN 1             // MEASURED: a positive pan angle turns the head to the LEFT

/* ===================================================================== SSCMA reply parser */
/* The camera answers over its I2C transport with JSON objects framed as "\r{...}\n":
 *   {"type": 0, "name": "INVOKE", "code": 0, "data": {...}}          response to a command
 *   {"type": 1, "name": "INVOKE", "code": 0, "data": {"count": 12,
 *       "perf": [7, 20, 1], "boxes": [[x, y, w, h, score, target], ...]}}   one result
 * Box x, y are the box centre in pixels of the 240 x 240 frame (MEASURED on its USB port).
 * Only the keys needed are looked at; nothing is allocated. */

enum { FP_MSG_NONE = 0, FP_MSG_RESPONSE = 1, FP_MSG_RESULT = 2, FP_MSG_OTHER = 3 };

struct FpMsg {
  uint8_t kind;      // FP_MSG_*
  int code;          // "code" of a response (0 = OK)
  char name[16];     // "name" (e.g. "INVOKE", "SENSOR", "BREAK")
  uint8_t has_box;   // result: at least one box
  int16_t x, y;      // result: best box centre, pixels from the image centre (x + = dog's right)
  uint16_t w, h;     // result: best box size, pixels
  uint8_t score;     // result: best box score, 0-100
  uint16_t perf_ms;  // result: sum of the camera's own preprocess + inference + postprocess
};

struct FpParser {
  char buf[FP_RX_BUF + 1];
  uint16_t len;
  uint32_t overflows;   // replies dropped because they did not fit in buf
  uint32_t bad_frames;  // frames without a type/name
};

static inline void fpParserReset(FpParser *p) {
  p->len = 0;
  p->buf[0] = '\0';
}

/* Room for new bytes; when full, keep only a frame start ("\r{") that is not at the front,
 * or drop everything (counted as an overflow). */
static inline uint16_t fpParserRoom(FpParser *p) {
  if (p->len >= FP_RX_BUF) {
    char *start = NULL;
    for (uint16_t i = 1; i + 1 < p->len; i++)
      if (p->buf[i] == '\r' && p->buf[i + 1] == '{') start = p->buf + i;
    p->overflows++;
    if (start) {
      uint16_t keep = (uint16_t)(p->len - (start - p->buf));
      memmove(p->buf, start, keep);
      p->len = keep;
    } else
      p->len = 0;
    p->buf[p->len] = '\0';
    if (p->len >= FP_RX_BUF) p->len = 0;  // one frame longer than the buffer: give up on it
  }
  return (uint16_t)(FP_RX_BUF - p->len);
}

static inline char *fpWriteSpot(FpParser *p) { return p->buf + p->len; }

static inline void fpParserAdd(FpParser *p, uint16_t n) {
  for (uint16_t i = 0; i < n && p->len + i < FP_RX_BUF; i++)  // a NUL byte would end strstr
    if (p->buf[p->len + i] == '\0') p->buf[p->len + i] = ' ';
  p->len = (uint16_t)(p->len + n);
  if (p->len > FP_RX_BUF) p->len = FP_RX_BUF;
  p->buf[p->len] = '\0';
}

static inline void fpParserFeed(FpParser *p, const char *data, uint16_t n) {
  while (n) {
    uint16_t room = fpParserRoom(p);
    uint16_t k = n < room ? n : room;
    memcpy(fpWriteSpot(p), data, k);
    fpParserAdd(p, k);
    data += k;
    n = (uint16_t)(n - k);
  }
}

/* value after "key": (skips spaces and the colon); NULL if the key is not in [s, end) */
static inline const char *fpAfterKey(const char *s, const char *key) {
  const char *k = strstr(s, key);
  if (!k) return NULL;
  k += strlen(key);
  while (*k == ' ' || *k == ':' || *k == '\t') k++;
  return k;
}

static inline long fpNum(const char **q) {  // integer at *q (a float is cut at its '.')
  char *end;
  long v = strtol(*q, &end, 10);
  while (*end && *end != ',' && *end != ']' && *end != '}') end++;
  *q = end;
  return v;
}

static inline void fpSkip(const char **q) {
  while (**q == ' ' || **q == ',' || **q == '\t' || **q == '\r' || **q == '\n') (*q)++;
}

/* Parse one NUL-terminated JSON object (without the \r and \n framing). */
static inline void fpParseObject(const char *obj, FpMsg *m) {
  const char *q;
  memset(m, 0, sizeof(*m));
  m->kind = FP_MSG_OTHER;
  int type = -1;
  if ((q = fpAfterKey(obj, "\"type\""))) type = (int)strtol(q, NULL, 10);
  if ((q = fpAfterKey(obj, "\"name\"")) && *q == '"') {
    q++;
    size_t i = 0;
    while (*q && *q != '"' && i + 1 < sizeof(m->name)) m->name[i++] = *q++;
    m->name[i] = '\0';
  }
  if ((q = fpAfterKey(obj, "\"code\""))) m->code = (int)strtol(q, NULL, 10);
  if (type == 0) {
    m->kind = FP_MSG_RESPONSE;
    return;
  }
  if (type != 1 || strcmp(m->name, "INVOKE") != 0) return;
  m->kind = FP_MSG_RESULT;
  if ((q = fpAfterKey(obj, "\"perf\"")) && *q == '[') {
    long sum = 0;
    q++;
    for (int i = 0; i < 3; i++) {
      fpSkip(&q);
      if (*q == ']' || !*q) break;
      sum += fpNum(&q);
    }
    m->perf_ms = (uint16_t)(sum < 0 ? 0 : (sum > 60000 ? 60000 : sum));
  }
  if ((q = fpAfterKey(obj, "\"boxes\"")) && *q == '[') {
    q++;
    int best = -1;
    for (;;) {
      fpSkip(&q);
      if (*q != '[') break;  // ']' (end of the list) or anything unexpected
      q++;
      long v[6] = {0, 0, 0, 0, 0, 0};
      int k = 0;
      for (; k < 6; k++) {
        fpSkip(&q);
        if (*q == ']' || !*q) break;
        v[k] = fpNum(&q);
      }
      while (*q && *q != ']') q++;
      if (*q == ']') q++;
      if (k >= 5 && v[4] >= FP_SCORE_MIN && v[4] > best) {
        best = (int)v[4];
        m->has_box = 1;
        m->x = (int16_t)(v[0] - FP_FRAME_PX / 2);
        m->y = (int16_t)(v[1] - FP_FRAME_PX / 2);
        m->w = (uint16_t)(v[2] < 0 ? 0 : v[2]);
        m->h = (uint16_t)(v[3] < 0 ? 0 : v[3]);
        m->score = (uint8_t)(v[4] < 0 ? 0 : (v[4] > 255 ? 255 : v[4]));
      }
    }
  }
}

/* Take the next complete "\r{...}\n" frame out of the buffer. Returns 1 and fills *m, or 0
 * if no complete frame is buffered. Bytes before a frame start are dropped. */
static inline int fpParserNext(FpParser *p, FpMsg *m) {
  for (;;) {
    char *start = strstr(p->buf, "\r{");
    if (!start) {  // keep a trailing '\r' (the start may be split), drop the rest
      if (p->len && p->buf[p->len - 1] == '\r') {
        p->buf[0] = '\r';
        p->len = 1;
      } else
        p->len = 0;
      p->buf[p->len] = '\0';
      return 0;
    }
    char *end = strstr(start + 2, "}\n");
    if (!end) {  // incomplete: move it to the front and wait for more
      if (start != p->buf) {
        uint16_t keep = (uint16_t)(p->len - (start - p->buf));
        memmove(p->buf, start, keep);
        p->len = keep;
        p->buf[p->len] = '\0';
      }
      return 0;
    }
    /* a new frame start before the end means the earlier one was cut: skip it */
    char *again = strstr(start + 2, "\r{");
    if (again && again < end) {
      uint16_t keep = (uint16_t)(p->len - (again - p->buf));
      memmove(p->buf, again, keep);
      p->len = keep;
      p->buf[p->len] = '\0';
      p->bad_frames++;
      continue;
    }
    end[1] = '\0';  // the object is start+1 .. end (the '}' stays)
    fpParseObject(start + 1, m);
    if (m->kind == FP_MSG_OTHER && !m->name[0]) p->bad_frames++;
    uint16_t used = (uint16_t)(end + 2 - p->buf);
    memmove(p->buf, end + 2, p->len - used);
    p->len = (uint16_t)(p->len - used);
    p->buf[p->len] = '\0';
    return 1;
  }
}

/* =================================================================== on-board control step */
struct FpControl {
  fp_state_t circuit;
  uint8_t started;          // a step has been made since fpControlReset
  uint32_t last_step_ms;    // time of the previous step
  uint32_t last_seen_ms;    // time of the last result with a face (or of the reset)
  float pan_deg;            // pan target, deg, + = left
  float turn_deg_s;         // circuit command, deg/s (+ = turn left)
  float cmd_deg_s;          // after the rate limit; 0 while holding
  float bearing_deg;        // corrected bearing given to the circuit (if a face)
  uint8_t holding;
  uint32_t hist_ms[FP_PAN_HISTORY];
  float hist_pan[FP_PAN_HISTORY];
  uint8_t hist_n, hist_head;
};

static inline float fpClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline void fpPanRecord(FpControl *c, uint32_t t_ms, float pan) {
  c->hist_head = (uint8_t)((c->hist_head + 1) % FP_PAN_HISTORY);
  c->hist_ms[c->hist_head] = t_ms;
  c->hist_pan[c->hist_head] = pan;
  if (c->hist_n < FP_PAN_HISTORY) c->hist_n++;
}

/* The pan target that was in force at time t_ms (the newest record not after t_ms; the oldest
 * known one if t_ms is before all of them). */
static inline float fpPanAt(const FpControl *c, uint32_t t_ms) {
  if (!c->hist_n) return c->pan_deg;
  uint8_t i = c->hist_head;
  float pan = c->hist_pan[i];
  for (uint8_t k = 0; k < c->hist_n; k++) {
    pan = c->hist_pan[i];
    if ((int32_t)(t_ms - c->hist_ms[i]) >= 0) return pan;
    i = (uint8_t)((i + FP_PAN_HISTORY - 1) % FP_PAN_HISTORY);
  }
  return pan;
}

static inline void fpControlReset(FpControl *c, uint32_t now_ms, float pan_now_deg) {
  fp_init(&c->circuit);
  c->started = 0;
  c->last_step_ms = c->last_seen_ms = now_ms;
  c->pan_deg = fpClamp(pan_now_deg, -FP_PAN_LIMIT_DEG, FP_PAN_LIMIT_DEG);
  c->turn_deg_s = c->cmd_deg_s = c->bearing_deg = 0.0f;
  c->holding = 0;
  c->hist_n = c->hist_head = 0;
  fpPanRecord(c, now_ms, c->pan_deg);
}

/* Bearing of a camera x (pixels from the image centre, + = dog's right), degrees, + = left.
 * MEASURED sign: a face on the dog's left gives negative x through this firmware's camera
 * setting (NOTES.md). */
static inline float fpBearingDeg(int16_t x) { return -(float)x * FP_FOV_DEG / (float)FP_FRAME_PX; }

/* One step for one camera result. t_meas_ms: when the camera took the picture (estimated);
 * now_ms: now. Returns the new pan target in degrees. */
static inline float fpControlStep(FpControl *c, uint8_t has_face, int16_t x, uint32_t t_meas_ms,
                                  uint32_t now_ms) {
  uint32_t gap = now_ms - c->last_step_ms;
  float dt = c->started ? gap / 1000.0f : 1.0f / 15.0f;
  if (c->started && gap > FP_RESET_GAP_MS) fp_reset(&c->circuit);
  dt = fpClamp(dt, FP_DT_MIN_S, FP_DT_MAX_S);
  c->started = 1;
  c->last_step_ms = now_ms;
  float az_rad = 0.0f;
  if (has_face) {
    c->last_seen_ms = now_ms;
    /* efference copy: the head turned by (pan now - pan when the picture was taken) since */
    c->bearing_deg = fpBearingDeg(x) - (c->pan_deg - fpPanAt(c, t_meas_ms));
    az_rad = c->bearing_deg * 0.01745329252f;
  }
  c->turn_deg_s = fp_step(&c->circuit, has_face, az_rad, dt, FP_AROUSAL) * 57.2957795131f;
  c->holding = !has_face && (now_ms - c->last_seen_ms) >= FP_HOLD_AFTER_MS;
  c->cmd_deg_s = c->holding ? 0.0f : fpClamp(c->turn_deg_s, -FP_HEAD_RATE_DEG_S, FP_HEAD_RATE_DEG_S);
  float pan_dt = dt < FP_PAN_DT_MAX_S ? dt : FP_PAN_DT_MAX_S;
  c->pan_deg = fpClamp(c->pan_deg + c->cmd_deg_s * pan_dt, -FP_PAN_LIMIT_DEG, FP_PAN_LIMIT_DEG);
  fpPanRecord(c, now_ms, c->pan_deg);
  return c->pan_deg;
}

/* ===================================================================== host line format */
/* "@FP <seq> <x> <y> <w> <h> <score>[ b=<deg>] t=<deg/s> p=<deg>[ hold];"  (README grammar)
 * The control fields are added only when on-board control is on. */
static inline int fpFormatResult(char *out, size_t size, uint16_t seq, const FpMsg *r,
                                 const FpControl *c) {
  int n;
  if (r->has_box)
    n = snprintf(out, size, "@FP %u %d %d %u %u %u", (unsigned)seq, (int)r->x, (int)r->y,
                 (unsigned)r->w, (unsigned)r->h, (unsigned)r->score);
  else
    n = snprintf(out, size, "@FP %u -", (unsigned)seq);
  if (c && n > 0 && (size_t)n < size) {
    if (r->has_box)
      n += snprintf(out + n, size - n, " b=%.1f", (double)c->bearing_deg);
    if ((size_t)n < size)
      n += snprintf(out + n, size - n, " t=%.1f p=%.1f%s", (double)c->cmd_deg_s,
                    (double)c->pan_deg, c->holding ? " hold" : "");
  }
  if (n > 0 && (size_t)n + 1 < size) {
    out[n++] = ';';
    out[n] = '\0';
  }
  return n;
}

#endif  // FLY_PURSUIT_CORE_H
