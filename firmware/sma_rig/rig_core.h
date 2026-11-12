// rig_core.h - hardware-independent logic of the SMA brake-duct bench rig.
//
// The Arduino sketch (sma_rig.ino) reads the sensors, fills an Inputs struct, calls Core::update() and
// drives the heater SSR, fan and LED from the Outputs struct. The same file is compiled for the PC
// (host/rig_core_capi.cpp) so the digital twin runs exactly this code.
//
// Serial commands (one per line, case-insensitive):
//   HEAT <pct>          manual heater duty (time-proportional, 200 ms window)
//   FAN <pct>           fan duty (25 kHz PWM)
//   HOLD <degC>         PI control of the spring temperature to a setpoint
//   RAMP <degC> <C/min> ramp the spring setpoint from the present temperature
//   LAP <n>             run the built-in braking-zone lap profile n times
//   COOL                heater off, fan 100 %
//   STOP                heater off, fan 30 %, idle
//   TARE | CAL <grams>  load-cell zero / span with a known mass
//   ZERO                louver angle zero (louvers closed)
//   LIMIT <degC>        spring over-temperature cut-out (default 110)
//   RESET               clear a latched fault
//   STATUS              print settings as a '#' comment line
// Telemetry: one JSON object per line every 50 ms (20 Hz), e.g.
//   {"timestamp_ms":1250,"temp_C":45.25,"force_N":8.10,"stroke_mm":0.00,"angle_deg":0.0,...}
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rig {

struct Inputs {
  uint32_t now_ms;
  float t_spring_c;      // TC1, thermocouple on the spring pack
  float t_air_c;         // TC2, plenum air
  float t_heater_c;      // TC3, reflector
  uint8_t t_spring_ok, t_air_ok, t_heater_ok;
  uint8_t load_ready;    // HX711 has a new sample
  int32_t load_raw;
  uint8_t angle_ok;
  uint16_t angle_raw;    // AS5600 0..4095
  uint32_t tach_count;   // cumulative fan tach pulses (2 per rev)
  uint16_t pitot_adc;    // 0..1023, MPXV7002DP
  uint8_t button;        // 1 = pressed (debounced)
};

struct Outputs {
  uint8_t heater_on;     // SSR level right now
  uint8_t fan_pct;
  uint8_t led_on;
};

enum Mode : uint8_t { MODE_IDLE = 0, MODE_MANUAL, MODE_HOLD, MODE_RAMP, MODE_LAP, MODE_COOL };
enum Fault : uint8_t { FAULT_NONE = 0, FAULT_TC = 1, FAULT_SPRING_OT = 2, FAULT_HEATER_OT = 4 };

class Core {
 public:
  void begin(uint32_t now_ms);
  void command(const char* line);
  void update(const Inputs& in, Outputs& out);
  bool telemetry(char* buf, size_t len);        // true when a 20 Hz line is due
  bool pending_message(char* buf, size_t len);  // '#' comment lines (acks, STATUS, faults)

  // constants (public so the twin and tests can read them)
  static constexpr float CRANK_R_MM = 10.89f;   // Phase 3 external crank
  static constexpr float CRANK_PHI0_DEG = -40.0f;
  static constexpr float HEATER_LIMIT_C = 350.0f;
  static constexpr uint16_t WINDOW_MS = 200;    // heater window: 10 mains cycles, 5 % steps with a zero-cross SSR
  static constexpr uint16_t TELEMETRY_MS = 50;

 private:
  void set_mode(Mode m);
  void message(const char* text);
  float stroke_from_angle(float deg) const;
  void run_pi(float dt_s, const Inputs& in);
  void run_lap(uint32_t now);

  Mode mode_ = MODE_IDLE;
  uint8_t fault_ = FAULT_NONE;
  float heater_pct_ = 0.0f;
  uint8_t fan_pct_ = 30;
  float setpoint_ = 25.0f, ramp_end_ = 25.0f, ramp_rate_ = 0.0f, integ_ = 0.0f;
  float spring_limit_ = 110.0f;
  // sensors -> engineering units
  int32_t load_offset_ = 0;
  float counts_per_n_ = 42800.0f;
  int32_t last_load_raw_ = 0;
  bool load_seen_ = false;
  bool tare_pending_ = true, zero_pending_ = true;
  uint16_t angle_zero_ = 0;
  float angle_deg_ = 0.0f;
  bool angle_seen_ = false;
  uint32_t pitot_sum_ = 0;
  uint16_t pitot_n_ = 0, pitot_zero_ = 512;
  float air_mps_ = 0.0f;
  float fan_rpm_ = 0.0f;
  uint32_t tach_prev_ = 0, rpm_t_ = 0;
  // lap profile state
  uint8_t lap_seg_ = 0, laps_left_ = 0;
  uint32_t seg_t0_ = 0;
  // timing
  uint32_t t0_ = 0, now_ = 0, last_update_ = 0, last_tel_ = 0, window_t0_ = 0;
  uint8_t prev_button_ = 0;
  Inputs last_in_{};
  char msg_[64];
  const char* msg_flash_ = nullptr;
  bool msg_pending_ = false, message_ram_ = false;
};

}  // namespace rig
