// rig_core.cpp - see rig_core.h. Portable C++ (avr-gcc 7.3 and MSVC), no heap, no printf floats.
#include "rig_core.h"

#include <math.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RS(s) PSTR(s)
#define RIG_FLASH PROGMEM
static inline char rs_read(const char* p) { return (char)pgm_read_byte(p); }
#else
#define RS(s) (s)
#define RIG_FLASH
static inline char rs_read(const char* p) { return *p; }
#endif
#include <stdlib.h>
#include <string.h>

namespace rig {

namespace {

// Lap profile {heater %, blower %} per 0.5 s step, generated from the car's simulated lap: heater duty
// reproduces the rotor radiation on the spring, blower duty the airflow at the spring (gen_lap_profile.py).
#include "lap_profile.h"

const float KP = 4.0f;      // %/K
const float KI = 0.12f;     // %/(K s)
const float DEG2RAD = 0.01745329252f;

bool finite_val(float x) { return x == x && x < 1e30f && x > -1e30f; }

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// --- tiny string builder (AVR printf has no %f) ---
struct Out {
  char* p;
  size_t left;
  void str(const char* s) {        // RAM string
    while (*s && left > 1) { *p++ = *s++; --left; }
    *p = 0;
  }
  void strf(const char* s) {       // flash string (RS literal)
    char c;
    while ((c = rs_read(s)) != 0 && left > 1) { *p++ = c; ++s; --left; }
    *p = 0;
  }
  void u32(uint32_t v) {
    char tmp[11];
    int n = 0;
    do { tmp[n++] = char('0' + v % 10); v /= 10; } while (v);
    while (n && left > 1) { *p++ = tmp[--n]; --left; }
    *p = 0;
  }
  void fix(float v, uint8_t dec) {
    if (!finite_val(v)) { strf(RS("null")); return; }
    if (v < 0) { str("-"); v = -v; }
    uint32_t scale = 1;
    for (uint8_t i = 0; i < dec; ++i) scale *= 10;
    uint32_t r = uint32_t(v * scale + 0.5f);
    u32(r / scale);
    if (dec) {
      str(".");
      uint32_t frac = r % scale;
      for (uint32_t d = scale / 10; d; d /= 10) {
        char c[2] = {char('0' + (frac / d) % 10), 0};
        str(c);
      }
    }
  }
  void key(const char* k) { strf(RS(",\"")); strf(k); strf(RS("\":")); }
};

bool word_is(const char* a, const char* b) {   // b is a flash string
  char cb;
  while (*a && *a != ' ' && (cb = rs_read(b)) != 0) {
    char ca = *a >= 'a' && *a <= 'z' ? char(*a - 32) : *a;
    if (ca != cb) return false;
    ++a; ++b;
  }
  return rs_read(b) == 0 && (*a == 0 || *a == ' ');
}

const char* arg(const char* line, uint8_t n) {
  const char* p = line;
  for (uint8_t i = 0; i < n; ++i) {
    while (*p && *p != ' ') ++p;
    while (*p == ' ') ++p;
  }
  return p;
}

const char* mode_name(Mode m) {
  switch (m) {
    case MODE_MANUAL: return RS("MANUAL");
    case MODE_HOLD: return RS("HOLD");
    case MODE_RAMP: return RS("RAMP");
    case MODE_LAP: return RS("LAP");
    case MODE_COOL: return RS("COOL");
    default: return RS("IDLE");
  }
}

}  // namespace

void Core::begin(uint32_t now_ms) {
  t0_ = now_ = last_update_ = last_tel_ = window_t0_ = rpm_t_ = now_ms;
  set_mode(MODE_IDLE);
  message(RS("# SMA rig firmware rev A ready"));
}

void Core::message(const char* text) {           // flash string, or RAM when ram = true
  message_ram_ = false;
  msg_flash_ = text;
  msg_pending_ = true;
}

void Core::set_mode(Mode m) {
  mode_ = m;
  integ_ = 0.0f;
  if (m == MODE_IDLE) { heater_pct_ = 0; fan_pct_ = 30; }
  if (m == MODE_COOL) { heater_pct_ = 0; fan_pct_ = 100; }
}

void Core::command(const char* line) {
  while (*line == ' ') ++line;
  float a1 = (float)atof(arg(line, 1));
  float a2 = (float)atof(arg(line, 2));
  if (word_is(line, RS("HEAT"))) {
    if (fault_) { message(RS("# refused: fault latched, send RESET")); return; }
    set_mode(MODE_MANUAL);
    heater_pct_ = clampf(a1, 0, 100);
  } else if (word_is(line, RS("FAN"))) {
    fan_pct_ = (uint8_t)clampf(a1, 0, 100);
  } else if (word_is(line, RS("HOLD"))) {
    if (fault_) { message(RS("# refused: fault latched, send RESET")); return; }
    set_mode(MODE_HOLD);
    setpoint_ = clampf(a1, 0, spring_limit_ - 5);
  } else if (word_is(line, RS("RAMP"))) {
    if (fault_) { message(RS("# refused: fault latched, send RESET")); return; }
    set_mode(MODE_RAMP);
    ramp_end_ = clampf(a1, 0, spring_limit_ - 5);
    ramp_rate_ = clampf(a2, 0.1f, 60.0f);
    setpoint_ = finite_val(last_in_.t_spring_c) ? last_in_.t_spring_c : 25.0f;
  } else if (word_is(line, RS("LAP"))) {
    if (fault_) { message(RS("# refused: fault latched, send RESET")); return; }
    set_mode(MODE_LAP);
    laps_left_ = (uint8_t)clampf(a1 < 1 ? 1 : a1, 1, 50);
    lap_seg_ = 0;
    seg_t0_ = now_;
  } else if (word_is(line, RS("COOL"))) {
    set_mode(MODE_COOL);
  } else if (word_is(line, RS("STOP"))) {
    set_mode(MODE_IDLE);
  } else if (word_is(line, RS("TARE"))) {
    tare_pending_ = true;
  } else if (word_is(line, RS("CAL"))) {
    float newton = a1 * 0.00980665f;
    if (newton > 0.1f && load_seen_) counts_per_n_ = float(last_load_raw_ - load_offset_) / newton;
  } else if (word_is(line, RS("ZERO"))) {
    zero_pending_ = true;
  } else if (word_is(line, RS("LIMIT"))) {
    spring_limit_ = clampf(a1, 40, 150);
  } else if (word_is(line, RS("RESET"))) {
    fault_ = FAULT_NONE;
    set_mode(MODE_IDLE);
  } else if (word_is(line, RS("STATUS"))) {
    char b[96];
    Out o{b, sizeof(b)};
    o.strf(RS("# mode "));
    o.strf(mode_name(mode_));
    o.strf(RS(" fault "));
    o.u32(fault_);
    o.strf(RS(" limit "));
    o.fix(spring_limit_, 1);
    o.strf(RS(" cal "));
    o.fix(counts_per_n_, 1);
    o.strf(RS(" zero "));
    o.u32(angle_zero_);
    strncpy(msg_, b, sizeof(msg_) - 1);
    msg_[sizeof(msg_) - 1] = 0;
    message_ram_ = true;
    msg_pending_ = true;
    return;
  } else {
    message(RS("# unknown command"));
    return;
  }
  message(RS("# ok"));
}

float Core::stroke_from_angle(float deg) const {
  // rod stroke from louver rotation through the external crank (Phase 3 kinematics)
  const float phi0 = CRANK_PHI0_DEG * DEG2RAD;
  return CRANK_R_MM * (sinf(deg * DEG2RAD + phi0) - sinf(phi0));
}

void Core::run_pi(float dt_s, const Inputs& in) {
  if (mode_ == MODE_RAMP) {
    float step = ramp_rate_ / 60.0f * dt_s;
    if (setpoint_ < ramp_end_) setpoint_ = setpoint_ + step > ramp_end_ ? ramp_end_ : setpoint_ + step;
    else if (setpoint_ > ramp_end_) setpoint_ = setpoint_ - step < ramp_end_ ? ramp_end_ : setpoint_ - step;
  }
  float err = setpoint_ - in.t_spring_c;
  float u = KP * err + integ_;
  if (u > 0.0f && u < 100.0f) integ_ += KI * err * dt_s;      // conditional-integration anti-windup
  else if ((u >= 100.0f && err < 0) || (u <= 0.0f && err > 0)) integ_ += KI * err * dt_s;
  integ_ = clampf(integ_, 0.0f, 100.0f);
  heater_pct_ = clampf(KP * err + integ_, 0.0f, 100.0f);
}

void Core::run_lap(uint32_t now) {
  while (now - seg_t0_ >= LAP_STEP_MS) {
    seg_t0_ += LAP_STEP_MS;
    if (++lap_seg_ >= LAP_STEPS) {
      lap_seg_ = 0;
      if (--laps_left_ == 0) { set_mode(MODE_COOL); message(RS("# lap profile done")); return; }
    }
  }
#ifdef __AVR__
  heater_pct_ = pgm_read_byte(&LAP_PROFILE[lap_seg_][0]);
  fan_pct_ = pgm_read_byte(&LAP_PROFILE[lap_seg_][1]);
#else
  heater_pct_ = LAP_PROFILE[lap_seg_][0];
  fan_pct_ = LAP_PROFILE[lap_seg_][1];
#endif
}

void Core::update(const Inputs& in, Outputs& out) {
  now_ = in.now_ms;
  float dt = float(now_ - last_update_) * 0.001f;
  last_update_ = now_;
  last_in_ = in;

  // ---- sensors to engineering units ----
  if (in.load_ready) {
    last_load_raw_ = in.load_raw;
    load_seen_ = true;
    if (tare_pending_) {
      // tare at the cold (closed) position: force_N is the actuation force above the bias-spring preload
      load_offset_ = in.load_raw;
      tare_pending_ = false;
    }
  }
  if (in.angle_ok) {
    if (zero_pending_) { angle_zero_ = in.angle_raw; zero_pending_ = false; }
    int32_t d = int32_t(in.angle_raw) - int32_t(angle_zero_);
    if (d < -2048) d += 4096;
    if (d > 2048) d -= 4096;
    angle_deg_ = float(d) * (360.0f / 4096.0f);
    angle_seen_ = true;
  }
  if (now_ - t0_ < 1000) {                    // pitot zero over the first second (fan at rest is not needed:
    pitot_sum_ += in.pitot_adc;                // MPXV7002 offset is taken before the fan spins up)
    ++pitot_n_;
    pitot_zero_ = uint16_t(pitot_sum_ / pitot_n_);
  } else {
    float dp = (float(in.pitot_adc) - float(pitot_zero_)) * (5000.0f / 1023.0f);   // Pa
    float v = dp > 0 ? sqrtf(2.0f * dp / 1.2f) : 0.0f;
    air_mps_ += (v - air_mps_) * clampf(dt / 0.5f, 0.0f, 1.0f);
  }
  if (now_ - rpm_t_ >= 1000) {
    fan_rpm_ = float(in.tach_count - tach_prev_) * 30000.0f / float(now_ - rpm_t_);
    tach_prev_ = in.tach_count;
    rpm_t_ = now_;
  }

  // ---- button: start a lap run when idle, stop otherwise ----
  if (in.button && !prev_button_) {
    if (mode_ == MODE_IDLE) command("LAP 1");
    else command("STOP");
  }
  prev_button_ = in.button;

  // ---- safety ----
  uint8_t f = fault_;
  if (!in.t_spring_ok || !in.t_heater_ok) f |= FAULT_TC;
  if (in.t_spring_ok && in.t_spring_c > spring_limit_) f |= FAULT_SPRING_OT;
  if (in.t_heater_ok && in.t_heater_c > HEATER_LIMIT_C) f |= FAULT_HEATER_OT;
  if (f != fault_) {
    fault_ = f;
    set_mode(MODE_COOL);
    message(RS("# FAULT: heater latched off, fan 100 %"));
  }

  // ---- control ----
  if (!fault_) {
    if (mode_ == MODE_HOLD || mode_ == MODE_RAMP) run_pi(dt, in);
    else if (mode_ == MODE_LAP) run_lap(now_);
  }
  if (fault_ || mode_ == MODE_IDLE || mode_ == MODE_COOL) heater_pct_ = 0;

  while (now_ - window_t0_ >= WINDOW_MS) window_t0_ += WINDOW_MS;
  out.heater_on = (float(now_ - window_t0_) < heater_pct_ * (WINDOW_MS / 100.0f)) ? 1 : 0;
  out.fan_pct = (now_ - t0_ < 1500) ? 0 : fan_pct_;   // fan held off while the pitot zero is taken
  out.led_on = fault_ ? uint8_t((now_ / 250) & 1) : uint8_t(heater_pct_ > 0 ? 1 : 0);
}

bool Core::telemetry(char* buf, size_t len) {
  if (now_ - last_tel_ < TELEMETRY_MS) return false;
  last_tel_ += TELEMETRY_MS;
  if (now_ - last_tel_ >= TELEMETRY_MS) last_tel_ = now_;      // don't burst after a stall
  const Inputs& in = last_in_;
  float force = load_seen_ ? float(last_load_raw_ - load_offset_) / counts_per_n_ : NAN;
  float stroke = angle_seen_ ? stroke_from_angle(angle_deg_) : NAN;
  Out o{buf, len};
  o.strf(RS("{\"timestamp_ms\":"));
  o.u32(now_ - t0_);
  o.key(RS("temp_C"));     o.fix(in.t_spring_ok ? in.t_spring_c : NAN, 2);
  o.key(RS("force_N"));    o.fix(force, 2);
  o.key(RS("stroke_mm"));  o.fix(stroke, 3);
  o.key(RS("angle_deg"));  o.fix(angle_deg_, 2);
  o.key(RS("t_air_C"));    o.fix(in.t_air_ok ? in.t_air_c : NAN, 2);
  o.key(RS("t_heater_C")); o.fix(in.t_heater_ok ? in.t_heater_c : NAN, 1);
  o.key(RS("heater_pct")); o.fix(heater_pct_, 1);
  o.key(RS("fan_pct"));    o.u32(fan_pct_);
  o.key(RS("fan_rpm"));    o.fix(fan_rpm_, 0);
  o.key(RS("air_mps"));    o.fix(air_mps_, 2);
  o.key(RS("setpoint_C")); o.fix((mode_ == MODE_HOLD || mode_ == MODE_RAMP) ? setpoint_ : NAN, 2);
  o.strf(RS(",\"mode\":\"")); o.strf(mode_name(mode_)); o.strf(RS("\""));
  o.key(RS("fault"));      o.u32(fault_);
  o.strf(RS("}"));
  return true;
}

bool Core::pending_message(char* buf, size_t len) {
  if (!msg_pending_) return false;
  if (message_ram_) {
    strncpy(buf, msg_, len - 1);
    buf[len - 1] = 0;
  } else {
    Out o{buf, len};
    o.strf(msg_flash_);
  }
  msg_pending_ = false;
  return true;
}

}  // namespace rig
