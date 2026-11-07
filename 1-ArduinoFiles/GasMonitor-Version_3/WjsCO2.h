#ifndef WJS_CO2_H
#define WJS_CO2_H

#include <Arduino.h>
#include <math.h>
#ifndef WJS_CO2_NO_EEPROM
#include <EEPROM.h>
#endif
#include "WjsDTypes.h"

/*
  DFRobot Gravity analog CO2 sensor V2.0 (SEN0159, MG811 element, on-board 8.5x amp + comparator) - NOT used in
  GasMonitor.ino by default (the IR sensor is). Pipeline: ADC -> Vout -> /gain -> T/RH correction -> log-linear curve
      V = V0 + s*(log10(ppm) - log10(ppm0)),   s = dV / (log10(ppm0) - log10(1000))

  Changes vs the original: implements IGasSensor; samples once per SAMPLE_MS with a settle read; floating-pin probe;
  5-sample debounce before ERROR; getReading() (clean "below baseline" is reported as the baseline value, it cannot
  be resolved); default EEPROM slot moved to 64 (the old default 0 collided with the MEMS slots); recalibrate() only
  uses valid samples.
  References: [1] MG811 datasheet  [2] DFRobot SEN0159 wiki (gain 8.5, zero 0.220 V, drop 0.030 V @1000 ppm)
              [3] ATmega ADC: Vin = ADC*Vref/1024  [4] NOAA GML outdoor baseline ~420 ppm.
  Power: needs the dedicated supply jack (heater); USB power alone is not enough (see DFRobot wiki).
*/

class WjsCO2 : public IGasSensor {
public:
  enum : int { PPM_BELOW_BASELINE = -1, PPM_NOT_READY = -2, PPM_FAULT = -3 };

  WjsCO2(uint8_t analogPin, uint8_t digitalPin, bool runInitialBake,
         float defaultZeroV = 0.220f, float dcGain = 8.5f, float reactionV = 0.030f,
         float baselinePPM = 400.0f, float vRef = 5.0f, float adcSteps = 1024.0f, int eepromAddr = 420)
    : _analogPin{analogPin}, _digitalPin{digitalPin}, _bakeOnFirstUse{runInitialBake}
    , _dcGain{dcGain}, _vRef{vRef}, _adcSteps{adcSteps}, _eepromAddr{eepromAddr}
    , _baselinePPM{baselinePPM}, _baselineLog{log10f(baselinePPM)}, _zeroV{defaultZeroV}, _reactionV{reactionV}
  { const float d = _baselineLog - 3.0f; _slope = (d < -0.01f) ? _reactionV / d : -0.0754f; }

  void begin()
  {
    pinMode(_digitalPin, INPUT_PULLUP);
    const bool stored = loadCalibration();
    _needCal = _bakeOnFirstUse && !stored;
    _warmupMs = _needCal ? FORTY_EIGHT_H_MS : FIFTEEN_MIN_MS;
    _startMs = _lastSampleMs = _lastProbeMs = millis();
    _seeded = false; _bad = 0; _fault = nullptr;
    _state = SensorState::STATE_WARMING_UP;
    if (Wjs::analogPinFloating(_analogPin, _adcSteps)) enterError(F("no sensor (pin floating)"));
  }

  SensorState updateAndGetState() override
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();
    if (_state == SensorState::STATE_ERROR)
    {
      if (now - _lastProbeMs >= RETRY_MS)
      {
        _lastProbeMs = now;
        if (!Wjs::analogPinFloating(_analogPin, _adcSteps) && voltageOk(readVoltage()))
        { _fault = nullptr; _seeded = false; _bad = 0; if (_needCal) startCalibration(); else { _startMs = now; _warmupMs = FIFTEEN_MIN_MS; _state = SensorState::STATE_WARMING_UP; } }
      }
      return _state;
    }
    if (_seeded && now - _lastSampleMs < SAMPLE_MS) return _state;
    _lastSampleMs = now;
    const float v = readVoltage();
    if (!voltageOk(v)) { if (++_bad >= 5) enterError(F("voltage out of range")); return _state; }
    _bad = 0; _vEma = _seeded ? _vEma + EMA_ALPHA * (v - _vEma) : v; _seeded = true;
    if (now - _lastProbeMs >= PROBE_MS) { _lastProbeMs = now; if (Wjs::analogPinFloating(_analogPin, _adcSteps)) { enterError(F("sensor disconnected")); return _state; } }

    switch (_state)
    {
      case SensorState::STATE_WARMING_UP: if (now - _startMs >= _warmupMs) { if (_needCal) startCalibration(); else _state = SensorState::STATE_READY; } break;
      case SensorState::STATE_CALIBRATING: calibrationStep(); break;
      case SensorState::STATE_READY:
      case SensorState::STATE_ALARM:       // comparator output on the board is active-low; threshold set with the on-board pot
        _state = (digitalRead(_digitalPin) == LOW) ? SensorState::STATE_ALARM : SensorState::STATE_READY; break;
      default: break;
    }
    return _state;
  }

  void setEnvironment(float tempC, float rhPct) { _tempC = tempC; _rh = rhPct; _hasEnv = true; }
  void setAlarmThreshold(float) override {}      // hardware comparator only (potentiometer on the board)
  uint32_t getWarmupRemainingMs() const override { return (_state == SensorState::STATE_WARMING_UP) ? Wjs::remainingMs(_startMs, _warmupMs) : 0UL; }

  /** Re-zero in clean air at baselinePPM. */
  bool recalibrate() override { if (!Wjs::isMeasuring(_state)) return false; _needCal = true; startCalibration(); return true; }
  bool eraseCalibration()
  {
#ifndef WJS_CO2_NO_EEPROM
    const uint16_t blank = 0xFFFF; EEPROM.put(_eepromAddr, blank); return true;
#else
    return false;
#endif
  }

  /** Second calibration point in a known gas (> baseline, e.g. 1000 ppm). */
  bool calibrateSlope(float knownPPM)
  {
    const float v = correctedVolts(), dLog = log10f(knownPPM) - _baselineLog;
    if (_state != SensorState::STATE_READY || dLog <= 0.05f || v >= _zeroV) return false;
    _slope = (v - _zeroV) / dLog; saveCalibration(); return true;
  }

  bool getReading(float& ppm) const override
  {
    if (!Wjs::isMeasuring(_state) || !_seeded) return false;
    const float v = correctedVolts();
    if (v >= _zeroV) { ppm = _baselinePPM; return true; }                 // at/below baseline: cannot be resolved
    const float p = powf(10.0f, (v - _zeroV) / _slope + _baselineLog);
    ppm = (p > MAX_PPM) ? MAX_PPM : p;
    return true;
  }
  /** Legacy API (int ppm or PPM_* codes). */
  int getPPM() const
  {
    if (_state == SensorState::STATE_ERROR) return PPM_FAULT;
    float p; if (!getReading(p)) return PPM_NOT_READY;
    return (correctedVolts() >= _zeroV) ? PPM_BELOW_BASELINE : int(p + 0.5f);
  }
  float getVoltage() const { return _seeded ? _vEma : NAN; }
  SensorState getState() const override { return _state; }
  const __FlashStringHelper* getFaultText() const override { return (_state == SensorState::STATE_ERROR) ? _fault : nullptr; }

  void getStatusLine(char* out, uint8_t size) const override
  {
    char a[12], b[12];
    if (_state == SensorState::STATE_ERROR) { strncpy(out, "FAULT: ", size); out[size - 1] = '\0'; if (_fault) { const uint8_t l = strlen(out); Wjs::copyFlash(out + l, size - l, _fault); } return; }
    if (_state == SensorState::STATE_WARMING_UP) { Wjs::fmtDuration(a, sizeof(a), getWarmupRemainingMs()); snprintf(out, size, "Warm-up left %s", a); return; }
    Wjs::fmtFloat(a, sizeof(a), getVoltage(), 3); Wjs::fmtFloat(b, sizeof(b), _zeroV, 4);
    snprintf(out, size, "V %s  V0 %s", a, b);
  }

  void printDashboard(Print& out) const override
  {
    float p;
    out.print(F("[")); out.print(Wjs::stateName(_state)); out.print(F(" CO2-MG811] V=")); Wjs::printNum(out, getVoltage(), 3);
    out.print(F(" V0=")); Wjs::printNum(out, _zeroV, 4); out.print(F(" CO2=")); Wjs::printNum(out, getReading(p) ? p : NAN, 0); out.print(F("ppm"));
    if (_state == SensorState::STATE_ERROR && _fault) { out.print(F(" FAULT: ")); out.print(_fault); }
    out.println();
  }

private:
  static constexpr uint32_t FORTY_EIGHT_H_MS{172800000UL}, FIFTEEN_MIN_MS{900000UL}, PROBE_MS{30000UL}, RETRY_MS{2000UL};
  static constexpr uint16_t SAMPLE_MS{100};
  static constexpr uint8_t  CAL_SAMPLES{30};
  static constexpr float    EMA_ALPHA{0.2f}, V_MARGIN{0.05f}, MAX_PPM{10000.0f};
  static constexpr float    T_REF{28.0f}, RH_REF{65.0f}, K_T{0.000125f}, K_RH{-0.00004f};   // MG811 datasheet dependency (approx.)
  static constexpr uint16_t EE_MAGIC{0xC02A};
  struct Record { uint16_t magic; float zero; float slope; uint8_t crc; };

  const uint8_t _analogPin, _digitalPin; const bool _bakeOnFirstUse; const float _dcGain, _vRef, _adcSteps; const int _eepromAddr;
  const float _baselinePPM, _baselineLog;
  float _zeroV, _reactionV, _slope{-0.0754f}, _tempC{T_REF}, _rh{RH_REF}, _vEma{0}, _calSum{0};
  uint32_t _warmupMs{FIFTEEN_MIN_MS}, _startMs{0}, _lastSampleMs{0}, _lastProbeMs{0};
  uint8_t _calN{0}, _bad{0}; bool _hasEnv{false}, _needCal{false}, _seeded{false};
  SensorState _state{SensorState::STATE_OFF}; const __FlashStringHelper* _fault{nullptr};

  float readVoltage() const { analogRead(_analogPin); uint32_t s = 0; for (uint8_t i = 0; i < 4; ++i) s += analogRead(_analogPin); return (s / 4.0f) * _vRef / _adcSteps; }
  bool  voltageOk(float v) const { return v > V_MARGIN && v < _vRef - V_MARGIN; }
  void  enterError(const __FlashStringHelper* why) { _state = SensorState::STATE_ERROR; _fault = why; _lastProbeMs = millis(); _bad = 0; }
  float correctedVolts() const { const float off = _hasEnv ? K_T * (_tempC - T_REF) + K_RH * (_rh - RH_REF) : 0.0f; return _vEma / _dcGain - off; }

  void startCalibration() { _calSum = 0.0f; _calN = 0; _state = SensorState::STATE_CALIBRATING; }
  void calibrationStep()
  {
    _calSum += correctedVolts();
    if (++_calN >= CAL_SAMPLES) { _zeroV = _calSum / CAL_SAMPLES; _needCal = false; saveCalibration(); _state = SensorState::STATE_READY; }
  }

  void saveCalibration() const
  {
#ifndef WJS_CO2_NO_EEPROM
    Record r; memset(&r, 0, sizeof(r)); r.magic = EE_MAGIC; r.zero = _zeroV; r.slope = _slope;
    r.crc = Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc)); EEPROM.put(_eepromAddr, r);
#endif
  }
  bool loadCalibration()
  {
#ifndef WJS_CO2_NO_EEPROM
    Record r; EEPROM.get(_eepromAddr, r);
    if (r.magic != EE_MAGIC || r.crc != Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc))) return false;
    if (!(r.zero > 0.05f && r.zero < 1.0f && r.slope < -0.005f && r.slope > -0.5f)) return false;
    _zeroV = r.zero; _slope = r.slope; return true;
#else
    return false;
#endif
  }
};

#endif
