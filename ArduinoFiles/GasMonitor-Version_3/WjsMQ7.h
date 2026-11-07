#ifndef WJS_MQ7_H
#define WJS_MQ7_H

#include <Arduino.h>
#include <math.h>
#ifndef WJS_MQ7_NO_EEPROM
#include <EEPROM.h>
#endif
#include "WjsDTypes.h"

/*
  MQ-7 semiconductor CO sensor (Hanwei / Pololu #1482) - NOT used in GasMonitor.ino by default.
  Heater cycle: 60 s @ 5.0 V (clean) -> 90 s @ 1.4 V (measure CO). Pipeline: ADC -> Rs -> Rs/R0 -> ppm

  Fixes vs the original driver
  * CURVE: the curve ppm = 100*(Rs/R0)^-1.53 is normalised to R0 = Rs at 100 ppm CO, but the old code calibrated R0 in
    CLEAN air - so clean air read ~100 ppm. R0 (clean air) is now divided by CLEAN_AIR_RATIO to get the 100 ppm
    reference. 27.5 is the value commonly used for the MQ-7 (e.g. MQUnifiedsensor); verify against your datasheet figure.
  * READING WINDOW: Rs is only meaningful at the end of the 1.4 V phase. The old driver evaluated ppm continuously,
    including during the 5 V purge. Rs is now averaged over the last MEASURE_WINDOW_MS of each 1.4 V phase and held
    (one new value per 150 s cycle).
  * Warm-up: 48 h on first use (datasheet), 10 min when R0 is stored. Old code went READY after 60 s with a made-up R0.
  * NaN/fault no longer reported as 0 ppm; floating-pin probe; R0 stored with CRC.
  * Heater PWM_LOW = 71 (1.39 V, datasheet 1.4 V +-0.1 V). Pins: the heater pin MUST be a PWM pin (Mega: 2-13, 44-46)
    and different from the analog pin (the old commented-out config used A3 for both).
  References: [1] Hanwei MQ-7 datasheet  [2] Pololu #1482
*/

class WjsMQ7 : public IGasSensor {
public:
  enum : int { PPM_NOT_READY = -2, PPM_FAULT = -3 };

  WjsMQ7(uint8_t analogPin, uint8_t heaterPwmPin, float rLoad = 10.0f,
         float vRef = 5.0f, float adcSteps = 1024.0f, int eepromAddr = 400)
    : _analogPin{analogPin}, _pwmPin{heaterPwmPin}, _rLoad{rLoad}, _vRef{vRef}, _adcSteps{adcSteps}, _eepromAddr{eepromAddr}
  {}

  void begin(bool forceFullPreheat = false)
  {
    pinMode(_analogPin, INPUT);
    pinMode(_pwmPin, OUTPUT);
    const uint32_t now = millis();
    _startMs = _phaseStartMs = _lastSampleMs = _lastProbeMs = now;
    _isHighHeat = true; analogWrite(_pwmPin, PWM_HIGH);
    _r0Air = 0.0f; _quick = false; _heldValid = false; _fault = nullptr; _bad = 0;
    if (!forceFullPreheat && loadR0()) { _quick = true; _warmupMs = QUICK_WARMUP_MS; } else _warmupMs = FULL_WARMUP_MS;
    _state = SensorState::STATE_WARMING_UP;
    if (Wjs::analogPinFloating(_analogPin, _adcSteps)) enterError(F("no sensor (pin floating)"));
  }

  SensorState updateAndGetState() override
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();
    manageHeaterCycle(now);                           // keep the heater cycle running even while in ERROR

    if (_state == SensorState::STATE_ERROR)
    {
      if (now - _lastProbeMs >= RETRY_MS)
      {
        _lastProbeMs = now;
        if (!Wjs::analogPinFloating(_analogPin, _adcSteps) && voltageOk(readVoltage()))
        { _fault = nullptr; _startMs = now; _warmupMs = _r0Air > 0.0f ? QUICK_WARMUP_MS : FULL_WARMUP_MS; _state = SensorState::STATE_WARMING_UP; }
      }
      return _state;
    }
    if (now - _lastSampleMs < SAMPLE_MS) return _state;
    _lastSampleMs = now;

    const float v = readVoltage();
    if (!voltageOk(v)) { if (++_bad >= 5) enterError(F("voltage out of range")); return _state; }
    _bad = 0; _vEma = _seeded ? _vEma + EMA_ALPHA * (v - _vEma) : v; _seeded = true;
    if (now - _lastProbeMs >= PROBE_MS) { _lastProbeMs = now; if (Wjs::analogPinFloating(_analogPin, _adcSteps)) { enterError(F("sensor disconnected")); return _state; } }

    // average Rs over the last MEASURE_WINDOW_MS of the low-heat phase and hold it
    if (!_isHighHeat && now - _phaseStartMs >= LOW_HEAT_MS - MEASURE_WINDOW_MS)
    { const float rs = rsCompensated(); if (!isnan(rs)) { _winSum += rs; ++_winN; } }

    switch (_state)
    {
      case SensorState::STATE_WARMING_UP:
        if (now - _startMs >= _warmupMs) { if (_r0Air > 0.0f && _heldValid) _state = SensorState::STATE_READY; else if (_r0Air <= 0.0f) { _calN = 0; _calSum = 0; _state = SensorState::STATE_CALIBRATING; } }
        break;
      case SensorState::STATE_CALIBRATING: break;     // progressed by finishWindow()
      case SensorState::STATE_READY:
      case SensorState::STATE_ALARM:
        if (_alarmPPM > 0.0f && _heldValid)
        {
          const float p = ppmFromRs(_rsHeld);
          if (_state == SensorState::STATE_READY && p >= _alarmPPM) _state = SensorState::STATE_ALARM;
          else if (_state == SensorState::STATE_ALARM && p < 0.9f * _alarmPPM) _state = SensorState::STATE_READY;
        }
        else if (_state == SensorState::STATE_ALARM && _alarmPPM <= 0.0f) _state = SensorState::STATE_READY;
        break;
      default: break;
    }
    return _state;
  }

  void setEnvironment(float tempC, float rhPct) { _tempC = tempC; _rh = rhPct; _hasEnv = true; }
  void setAlarmThreshold(float ppm) override { _alarmPPM = ppm; }
  void setWarmupMs(uint32_t ms) { _warmupMs = ms; }

  /** Re-zero in CLEAN air: averages 3 measurement windows (~7.5 min). */
  bool recalibrate() override
  {
    if (!Wjs::isMeasuring(_state)) return false;
    _calSum = 0.0f; _calN = 0; _state = SensorState::STATE_CALIBRATING; return true;
  }
  bool eraseCalibration()
  {
#ifndef WJS_MQ7_NO_EEPROM
    const uint16_t blank = 0xFFFF; EEPROM.put(_eepromAddr, blank); return true;
#else
    return false;
#endif
  }

  bool getReading(float& ppm) const override
  {
    if (!Wjs::isMeasuring(_state) || !_heldValid || _r0Air <= 0.0f) return false;
    ppm = ppmFromRs(_rsHeld);
    return !isnan(ppm);
  }
  /** Legacy API: ppm or PPM_NOT_READY / PPM_FAULT. */
  float getPPM() const
  {
    if (_state == SensorState::STATE_ERROR) return float(PPM_FAULT);
    float p; return getReading(p) ? p : float(PPM_NOT_READY);
  }
  float getVoltage() const { return _seeded ? _vEma : NAN; }
  float getR0() const { return _r0Air; }
  bool  isMeasuringPhase() const { return !_isHighHeat; }
  SensorState getState() const override { return _state; }
  const __FlashStringHelper* getFaultText() const override { return (_state == SensorState::STATE_ERROR) ? _fault : nullptr; }
  uint32_t getWarmupRemainingMs() const override { return (_state == SensorState::STATE_WARMING_UP) ? Wjs::remainingMs(_startMs, _warmupMs) : 0UL; }

  void getStatusLine(char* out, uint8_t size) const override
  {
    char a[12], b[12];
    if (_state == SensorState::STATE_ERROR) { strncpy(out, "FAULT: ", size); out[size - 1] = '\0'; if (_fault) { const uint8_t l = strlen(out); Wjs::copyFlash(out + l, size - l, _fault); } return; }
    if (_state == SensorState::STATE_WARMING_UP) { Wjs::fmtDuration(a, sizeof(a), getWarmupRemainingMs()); snprintf(out, size, "Warm-up left %s", a); return; }
    Wjs::fmtFloat(a, sizeof(a), getVoltage(), 3); Wjs::fmtFloat(b, sizeof(b), _r0Air, 2);
    snprintf(out, size, "%s  V %s  R0air %sk", _isHighHeat ? "PURGE 5V" : "MEASURE 1.4V", a, b);
  }

  void printDashboard(Print& out) const override
  {
    float p; const bool ok = getReading(p);
    out.print(F("[")); out.print(Wjs::stateName(_state)); out.print(F(" MQ-7 CO | "));
    out.print(_isHighHeat ? F("PURGE 5V") : F("MEASURE 1.4V")); out.print(F("] V=")); Wjs::printNum(out, getVoltage(), 3);
    out.print(F(" R0air=")); Wjs::printNum(out, _r0Air, 2); out.print(F("k ppm=")); Wjs::printNum(out, ok ? p : NAN, 1);
    if (_state == SensorState::STATE_WARMING_UP) { out.print(F(" warmup-left=")); Wjs::printHM(out, getWarmupRemainingMs()); }
    if (_state == SensorState::STATE_ERROR && _fault) { out.print(F(" FAULT: ")); out.print(_fault); }
    out.println();
  }

private:
  static constexpr uint8_t  PWM_HIGH = 255, PWM_LOW = 71;                 // 5.0 V / ~1.39 V average
  static constexpr uint32_t HIGH_HEAT_MS = 60000UL, LOW_HEAT_MS = 90000UL, MEASURE_WINDOW_MS = 5000UL;
  static constexpr uint32_t FULL_WARMUP_MS = 172800000UL;                 // 48 h
  static constexpr uint32_t QUICK_WARMUP_MS = 600000UL;                   // 10 min with stored R0
  static constexpr uint32_t SAMPLE_MS = 100, PROBE_MS = 30000UL, RETRY_MS = 2000UL;
  static constexpr float    EMA_ALPHA = 0.3f, V_MARGIN = 0.05f;
  static constexpr float    CLEAN_AIR_RATIO = 27.5f;                      // Rs(clean air) / Rs(100 ppm CO) - verify
  static constexpr float    MAX_PPM = 2000.0f;                            // datasheet 20-2000 ppm
  static constexpr uint16_t EE_MAGIC = 0x4D37;
  struct Record { uint16_t magic; float r0; uint8_t crc; };

  const uint8_t _analogPin, _pwmPin; const float _rLoad, _vRef, _adcSteps; const int _eepromAddr;
  SensorState _state{SensorState::STATE_OFF}; const __FlashStringHelper* _fault{nullptr};
  float _vEma{0}, _r0Air{0}, _rsHeld{0}, _winSum{0}, _calSum{0}, _alarmPPM{0}, _tempC{20.0f}, _rh{65.0f};
  uint32_t _warmupMs{FULL_WARMUP_MS}, _startMs{0}, _phaseStartMs{0}, _lastSampleMs{0}, _lastProbeMs{0};
  uint16_t _winN{0}; uint8_t _calN{0}, _bad{0};
  bool _isHighHeat{true}, _seeded{false}, _hasEnv{false}, _heldValid{false}, _quick{false};

  float readVoltage() const { analogRead(_analogPin); uint32_t s = 0; for (uint8_t i = 0; i < 4; ++i) s += analogRead(_analogPin); return (s / 4.0f) * _vRef / _adcSteps; }
  bool  voltageOk(float v) const { return v > V_MARGIN && v < _vRef - V_MARGIN; }
  void  enterError(const __FlashStringHelper* why) { _state = SensorState::STATE_ERROR; _fault = why; _lastProbeMs = millis(); _bad = 0; }

  float rsCompensated() const
  {
    if (!_seeded) return NAN;
    const float rs = _rLoad * (_vRef - _vEma) / _vEma;
    if (!_hasEnv) return rs;
    const float k = 1.0f - 0.008f * (_tempC - 20.0f) - 0.003f * (_rh - 65.0f);   // approximate, ref 20 C / 65 %RH
    return (k > 0.1f) ? rs / k : rs;
  }
  float ppmFromRs(float rs) const
  {
    const float r0_100 = _r0Air / CLEAN_AIR_RATIO;                        // Rs at 100 ppm CO
    if (r0_100 <= 0.0f || rs <= 0.0f) return NAN;
    const float ppm = 100.0f * powf(rs / r0_100, -1.53f);
    return ppm > MAX_PPM ? MAX_PPM : ppm;
  }

  void manageHeaterCycle(uint32_t now)
  {
    if (_isHighHeat && now - _phaseStartMs >= HIGH_HEAT_MS)
    { _isHighHeat = false; _phaseStartMs = now; _winSum = 0; _winN = 0; analogWrite(_pwmPin, PWM_LOW); }
    else if (!_isHighHeat && now - _phaseStartMs >= LOW_HEAT_MS)
    { finishWindow(); _isHighHeat = true; _phaseStartMs = now; analogWrite(_pwmPin, PWM_HIGH); }
  }

  void finishWindow()
  {
    if (_winN == 0) return;
    _rsHeld = _winSum / _winN; _heldValid = true;
    if (_state == SensorState::STATE_CALIBRATING)
    {
      _calSum += _rsHeld;
      if (++_calN >= 3) { _r0Air = _calSum / _calN; saveR0(); _state = SensorState::STATE_READY; }
    }
    else if (_state == SensorState::STATE_WARMING_UP && _r0Air > 0.0f && millis() - _startMs >= _warmupMs) _state = SensorState::STATE_READY;
  }

  void saveR0() const
  {
#ifndef WJS_MQ7_NO_EEPROM
    Record r; memset(&r, 0, sizeof(r)); r.magic = EE_MAGIC; r.r0 = _r0Air; r.crc = Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), 6); EEPROM.put(_eepromAddr, r);
#endif
  }
  bool loadR0()
  {
#ifndef WJS_MQ7_NO_EEPROM
    Record r; EEPROM.get(_eepromAddr, r);
    if (r.magic != EE_MAGIC || r.crc != Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), 6) || !(r.r0 > 0.5f && r.r0 < 2000.0f)) return false;
    _r0Air = r.r0; return true;
#else
    return false;
#endif
  }
};

#endif // WJS_MQ7_H
