#ifndef WJSDTYPES_H
#define WJSDTYPES_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*
  Shared types and helpers for all Wjs* sensor drivers.

  SensorState : the state every sensor is in (OFF, WARMING_UP, CALIBRATING, READY, ALARM, ERROR)
  GasUnit     : how a concentration is DISPLAYED (ppm or vol%). Inside the code everything is ppm.
  ParamInfo   : description of ONE adjustable constant (name, unit, allowed range, decimals)
  IGasSensor  : the "contract" every gas sensor class follows. Because they all look the same from the
                outside, the screen code (NextionUI.h) can treat every sensor identically.
*/

enum class SensorState : uint8_t {
  STATE_OFF,          // begin() not called yet
  STATE_WARMING_UP,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_ALARM,
  STATE_ERROR         // not connected / bad data. Drivers retry automatically.
};

enum class GasUnit : uint8_t { PPM = 0, VOL_PERCENT = 1 };     // 1 vol% = 10 000 ppm

/** One adjustable constant, as shown on a Nextion settings row. */
struct ParamInfo {
  const __FlashStringHelper* label;   // up to 12 characters (fits the Nextion label box)
  const __FlashStringHelper* unit;    // up to 5 characters
  float   minValue;                   // values typed on the screen are clamped to this range
  float   maxValue;
  uint8_t decimals;                   // digits after the decimal point on the screen
};

namespace Wjs {

constexpr float PPM_PER_VOL_PERCENT = 10000.0f;

inline bool isMeasuring(SensorState s) {
  return s == SensorState::STATE_READY || s == SensorState::STATE_ALARM;
}

inline const __FlashStringHelper* stateName(SensorState s) {
  switch (s) {
    case SensorState::STATE_OFF:         return F("OFF");
    case SensorState::STATE_WARMING_UP:  return F("WARMUP");
    case SensorState::STATE_CALIBRATING: return F("CALIB");
    case SensorState::STATE_READY:       return F("READY");
    case SensorState::STATE_ALARM:       return F("ALARM");
    case SensorState::STATE_ERROR:       return F("ERROR");
  }
  return F("?");
}

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/** Copy a flash string into a RAM buffer (always NUL-terminated). */
inline void copyFlash(char* dst, uint8_t size, const __FlashStringHelper* src) {
  if (size == 0) return;
  strncpy_P(dst, reinterpret_cast<PGM_P>(src), size - 1);
  dst[size - 1] = '\0';
}

/** Float -> text. "n/a" for NaN/inf. Works on AVR (snprintf has no %f there). size must be >= 14. */
inline void fmtFloat(char* buf, uint8_t size, float v, uint8_t decimals) {
  if (isnan(v) || isinf(v)) { strncpy(buf, "n/a", size); buf[size - 1] = '\0'; return; }
  dtostrf(v, 1, decimals, buf);
  buf[size - 1] = '\0';
}

/** Text typed on the screen -> float. Accepts "12.5", " 12,5 ", "-3". Returns false for empty/garbage. */
inline bool parseFloat(const char* s, float& out) {
  while (*s == ' ') ++s;
  char tmp[24]; uint8_t n = 0;
  for (; *s && n < sizeof(tmp) - 1; ++s, ++n) tmp[n] = (*s == ',') ? '.' : *s;   // allow decimal comma
  tmp[n] = '\0';
  if (n == 0) return false;
  char* end = nullptr;
  const double v = strtod(tmp, &end);
  while (end && *end == ' ') ++end;
  if (end == tmp || (end && *end != '\0')) return false;
  if (isnan(v) || isinf(v)) return false;
  out = float(v);
  return true;
}

inline void printNum(Print& out, float v, uint8_t decimals) {
  if (isnan(v) || isinf(v)) out.print(F("n/a")); else out.print(v, decimals);
}

/** Print "12h 05m". */
inline void printHM(Print& out, uint32_t ms) {
  const uint32_t s = ms / 1000UL;
  out.print(s / 3600UL); out.print(F("h "));
  const uint32_t m = (s % 3600UL) / 60UL;
  if (m < 10) out.print('0');
  out.print(m); out.print('m');
}

/** ms -> "12h05m" (>= 1 h) or "14m05s" (buf >= 10 chars). */
inline void fmtDuration(char* buf, uint8_t size, uint32_t ms) {
  const uint32_t s = ms / 1000UL;
  if (s >= 3600UL) snprintf(buf, size, "%luh%02lum", (unsigned long)(s / 3600UL), (unsigned long)((s % 3600UL) / 60UL));
  else             snprintf(buf, size, "%lum%02lus", (unsigned long)(s / 60UL), (unsigned long)(s % 60UL));
}

inline uint32_t remainingMs(uint32_t startMs, uint32_t durationMs) {
  const uint32_t e = millis() - startMs;
  return (e >= durationMs) ? 0UL : (durationMs - e);
}

/**
  Detects an unconnected (floating) analog input by enabling the internal pull-up for ~2 ms.
  A connected sensor board holds the pin low/mid; a floating pin is pulled to ~Vcc.
  `pin` must be the Arduino pin constant (A0...), not the raw channel number.
*/
inline bool analogPinFloating(uint8_t pin, float adcSteps) {
  pinMode(pin, INPUT_PULLUP);
  delay(2);
  analogRead(pin);                          // discard: lets the ADC sample-and-hold settle
  const int a = analogRead(pin);
  pinMode(pin, INPUT);
  return a >= int(adcSteps * 0.975f);
}

inline uint8_t crc8(const uint8_t* d, uint8_t n) {
  uint8_t c = 0xA5;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; ++i) c = (c & 0x80) ? uint8_t((c << 1) ^ 0x07) : uint8_t(c << 1);
  }
  return c;
}

} // namespace Wjs

/** Interface implemented by every concentration sensor (MEMS, H2S, IR CO2, ...). */
class IGasSensor {
public:
  // ---------------------------------------------------------------- core (must be implemented)
  virtual SensorState updateAndGetState() = 0;
  virtual SensorState getState() const = 0;
  /** Concentration in ppm. true only while READY/ALARM and the value is valid. */
  virtual bool getReading(float& ppm) const = 0;
  /** ppm threshold; 0 disables. Drivers apply 10 % hysteresis. */
  virtual void setAlarmThreshold(float ppm) = 0;
  virtual uint32_t getWarmupRemainingMs() const = 0;
  /** nullptr when healthy. */
  virtual const __FlashStringHelper* getFaultText() const = 0;
  /** One short live line for the screen/serial, e.g. "V 2.31 Rs 9.3k ratio 1.00" or "Warm-up left 3h10m". */
  virtual void getStatusLine(char* out, uint8_t size) const = 0;
  virtual void printDashboard(Print& out) const = 0;

  // ---------------------------------------------------------------- optional
  /** Re-zero in clean air (MEMS). Returns false when unsupported / not allowed in the current state. */
  virtual bool recalibrate() { return false; }

  // ---------------------------------------------------------------- adjustable constants
  // The screen shows every constant a sensor offers, lets you edit them, and writes them back with setParam().
  virtual uint8_t paramCount() const { return 0; }
  virtual bool    paramInfo(uint8_t /*i*/, ParamInfo& /*p*/) const { return false; }
  virtual float   getParam(uint8_t /*i*/) const { return 0.0f; }
  /** Applies a new value (clamped to the allowed range). Returns false if i is out of range. */
  virtual bool    setParam(uint8_t /*i*/, float /*v*/) { return false; }
  /** Back to the built-in defaults (a stored R0 is kept). */
  virtual void    resetParamDefaults() {}
  /** true ONCE after the driver changed one of its own constants (e.g. finished a zero calibration). */
  virtual bool    consumeParamsChanged() { return false; }

  // ---------------------------------------------------------------- display units (ppm / vol%)
  virtual bool    supportsVolPercent() const { return false; }
  virtual uint8_t ppmDecimals() const { return 1; }             // decimals shown when the unit is ppm

  void    setUnit(GasUnit u) { _unit = (u == GasUnit::VOL_PERCENT && !supportsVolPercent()) ? GasUnit::PPM : u; }
  GasUnit getUnit() const { return _unit; }
  const __FlashStringHelper* unitLabel() const { return _unit == GasUnit::VOL_PERCENT ? F("vol%") : F("ppm"); }
  uint8_t displayDecimals() const { return _unit == GasUnit::VOL_PERCENT ? 3 : ppmDecimals(); }
  float   ppmToDisplay(float ppm) const { return _unit == GasUnit::VOL_PERCENT ? ppm / Wjs::PPM_PER_VOL_PERCENT : ppm; }
  float   displayToPpm(float v)   const { return _unit == GasUnit::VOL_PERCENT ? v * Wjs::PPM_PER_VOL_PERCENT : v; }
  /** Concentration in the chosen display unit. */
  bool    getReadingInUnit(float& value) const { float p; if (!getReading(p)) return false; value = ppmToDisplay(p); return true; }

protected:
  GasUnit _unit{GasUnit::PPM};
};

#endif // WJSDTYPES_H
