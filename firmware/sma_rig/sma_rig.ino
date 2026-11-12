// sma_rig.ino - Arduino UNO R3 firmware for the SMA brake-duct bench rig (sensor shield rev A).
// Hardware layer only: sensors in, SSR / fan / LED out. All logic lives in rig_core.{h,cpp}.
//
// Pins (shield schematic):
//   D13 SCK, D12 MISO, D10/D8/D7  MAX31855 CS for TC1 spring / TC2 air / TC3 reflector
//   A4 SDA, A5 SCL                AS5600 louver angle
//   D4 DT, D5 SCK                 HX711 load cell
//   D9 fan PWM 25 kHz (Timer1), D2 fan tach (INT0), D6 heater SSR, D3 button, A2 LED, A0 pitot
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31855.h>
#include <AS5600.h>
#include <HX711.h>

#include "rig_core.h"

const uint8_t PIN_CS_TC1 = 10, PIN_CS_TC2 = 8, PIN_CS_TC3 = 7;
const uint8_t PIN_HX_DT = 4, PIN_HX_SCK = 5;
const uint8_t PIN_FAN_PWM = 9, PIN_TACH = 2, PIN_SSR = 6, PIN_BTN = 3;
const uint8_t PIN_LED = A2, PIN_PITOT = A0;

Adafruit_MAX31855 tc1(PIN_CS_TC1), tc2(PIN_CS_TC2), tc3(PIN_CS_TC3);
AS5600 encoder;
HX711 loadcell;
rig::Core core;
rig::Inputs in;
rig::Outputs out;

volatile uint32_t tachPulses = 0;
void onTach() { tachPulses++; }

char line[48];
uint8_t lineLen = 0;
char tx[240];

void fanBegin() {
  pinMode(PIN_FAN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);               // fast PWM, TOP = ICR1, non-inverting on OC1A (D9)
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);    // no prescaler
  ICR1 = 639;                                       // 16 MHz / 640 = 25 kHz (Intel 4-pin fan spec)
  OCR1A = 0;
}

void fanSet(uint8_t pct) { OCR1A = (uint16_t)((uint32_t)pct * 639u / 100u); }

void readThermocouples() {
  static uint32_t last = 0;
  if (millis() - last < 100) return;                // MAX31855 converts at ~10 Hz
  last = millis();
  double t;
  t = tc1.readCelsius(); in.t_spring_ok = !isnan(t); if (in.t_spring_ok) in.t_spring_c = (float)t;
  t = tc2.readCelsius(); in.t_air_ok = !isnan(t);    if (in.t_air_ok) in.t_air_c = (float)t;
  t = tc3.readCelsius(); in.t_heater_ok = !isnan(t); if (in.t_heater_ok) in.t_heater_c = (float)t;
}

void readButton() {
  static uint8_t stable = 0, last = 0;
  static uint32_t changed = 0;
  uint8_t raw = digitalRead(PIN_BTN) == LOW;
  if (raw != last) { last = raw; changed = millis(); }
  if (millis() - changed > 30) stable = raw;
  in.button = stable;
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = 0; core.command(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_TACH, INPUT);                          // 10k pull-up on the shield
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), onTach, FALLING);
  fanBegin();
  tc1.begin(); tc2.begin(); tc3.begin();
  Wire.begin();
  encoder.begin();
  loadcell.begin(PIN_HX_DT, PIN_HX_SCK);
  delay(250);                                        // MAX31855 first conversion
  core.begin(millis());
}

void loop() {
  pollSerial();
  readThermocouples();
  readButton();
  in.load_ready = loadcell.is_ready();
  if (in.load_ready) in.load_raw = (int32_t)loadcell.read();
  in.angle_ok = encoder.isConnected();
  if (in.angle_ok) in.angle_raw = encoder.readAngle();
  noInterrupts();
  in.tach_count = tachPulses;
  interrupts();
  in.pitot_adc = analogRead(PIN_PITOT);
  in.now_ms = millis();

  core.update(in, out);
  digitalWrite(PIN_SSR, out.heater_on ? HIGH : LOW);
  fanSet(out.fan_pct);
  digitalWrite(PIN_LED, out.led_on ? HIGH : LOW);

  if (core.pending_message(tx, sizeof(tx))) Serial.println(tx);
  if (core.telemetry(tx, sizeof(tx))) Serial.println(tx);
}
