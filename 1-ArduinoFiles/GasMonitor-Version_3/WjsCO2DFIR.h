#ifndef WJS_IR_CO2_H
#define WJS_IR_CO2_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "WjsDTypes.h"

/*
  WjsIRCO2: DFRobot SEN0220 UART infrared CO2 sensor (NDIR, 0-50,000 ppm, based on the MH-Z16 protocol).
  Pipeline: poll command -> 9-byte frame -> checksum -> hi/lo bytes -> smoothing -> state machine

  ADJUSTABLE CONSTANTS (paramCount() = 3), edited from the Nextion settings page:
    0 Zero offset : ppm subtracted from the sensor's reading (software zero trim)
    1 Span factor : multiplier applied after the zero offset (1.0000 = off)
    2 Smoothing   : 0.02..1.00, weight of each new frame in the moving average (1 = no smoothing)
    final ppm = max(0, (smoothed raw - zero) * span).  The screen can show ppm or vol% (10 000 ppm = 1 vol%).

  Fixes vs the original driver
  * CHECKSUM: the original computed 256 - sum + 1, which rejects every valid frame (the command's own checksum
    0x79 = 256 - (0x01 + 0x86) proves the right formula is 256 - sum). The sensor therefore always read 0 ppm.
  * Reply timeout -> ERROR (a missing sensor used to read 0 ppm "READY").
  * Frame parser state is per object (was function-static) and re-syncs on 0xFF.
  * Light smoothing: the wiki FAQ says the output normally fluctuates.

  References
  [1] DFRobot SEN0220 wiki: 9600 baud, command FF 01 86 00 00 00 00 00 79, ppm = byte2*256 + byte3,
      3 min preheat (readings inside the first 3 minutes are not accurate), 4.5-5.5 V, <85 mA.
  [2] Winsen MH-Z16 manual (frame layout).
  Wiring on a Mega: sensor TX -> RX1 (pin 19), sensor RX -> TX1 (pin 18), common GND.
*/

class WjsIRCO2 : public IGasSensor {
public:
  enum : int { PPM_NOT_READY = -2, PPM_FAULT = -3 };

  /** @param runInitialBake false = skip the 3 minute preheat. */
  WjsIRCO2(HardwareSerial& serialPort, bool runInitialBake = true)
    : _serial{serialPort}, _warmupMs{runInitialBake ? uint32_t(THREE_MIN_MS) : uint32_t(0)}
  {}

  bool begin(unsigned long baudRate = 9600)
  {
    _serial.begin(baudRate);
    while (_serial.available() > 0) _serial.read();
    const uint32_t now = millis();
    _startMs = _lastPollMs = _lastFrameMs = now;
    _idx = 0; _bad = 0; _haveFrame = false; _fault = nullptr;
    _state = (_warmupMs > 0UL) ? SensorState::STATE_WARMING_UP : SensorState::STATE_READY;
    return true;
  }

  SensorState updateAndGetState() override
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();
    const bool gotFrame = service(now);

    if (_state == SensorState::STATE_ERROR)
    {
      if (gotFrame)                                   // sensor answers again; it may have been unpowered
      {
        _fault = nullptr; _startMs = now;
        _state = (_warmupMs > 0UL) ? SensorState::STATE_WARMING_UP : SensorState::STATE_READY;
      }
      return _state;
    }

    if (now - _lastFrameMs >= REPLY_TIMEOUT_MS)       // no valid frame for too long
    {
      _state = SensorState::STATE_ERROR; _haveFrame = false;
      _fault = (_bad > 0) ? F("bad frames (checksum)") : F("no reply (check TX/RX)");
      return _state;
    }

    switch (_state)
    {
      case SensorState::STATE_WARMING_UP:
        if (now - _startMs >= _warmupMs) _state = SensorState::STATE_READY;
        break;
      case SensorState::STATE_CALIBRATING:
        _state = SensorState::STATE_READY;
        break;
      case SensorState::STATE_READY:
        if (_alarmPPM > 0.0f && _haveFrame && current() >= _alarmPPM) _state = SensorState::STATE_ALARM;
        break;
      case SensorState::STATE_ALARM:
        if (_alarmPPM <= 0.0f || current() < 0.9f * _alarmPPM) _state = SensorState::STATE_READY;
        break;
      default: break;
    }
    return _state;
  }

  void setAlarmThreshold(float ppm) override { _alarmPPM = ppm; }
  void setWarmupMs(uint32_t ms) { _warmupMs = ms; }

  /** Legacy API: ppm, or PPM_NOT_READY / PPM_FAULT codes. Prefer getReading(). */
  float getPPM() const
  {
    if (_state == SensorState::STATE_ERROR) return float(PPM_FAULT);
    if (!Wjs::isMeasuring(_state))          return float(PPM_NOT_READY);
    return current();
  }

  bool getReading(float& ppm) const override
  {
    if (!Wjs::isMeasuring(_state) || !_haveFrame) return false;
    ppm = current();
    return true;
  }

  /** Last value even while preheating (not accurate during the first 3 min). NAN before the first frame. */
  float getLivePPM() const { return _haveFrame ? current() : NAN; }

  SensorState getState() const override { return _state; }
  const __FlashStringHelper* getFaultText() const override { return (_state == SensorState::STATE_ERROR) ? _fault : nullptr; }
  uint32_t getWarmupRemainingMs() const override
  {
    return (_state == SensorState::STATE_WARMING_UP) ? Wjs::remainingMs(_startMs, _warmupMs) : 0UL;
  }
  unsigned long getTimeElapsed() const { return (_state == SensorState::STATE_OFF) ? 0UL : (millis() - _startMs); }

  void getStatusLine(char* out, uint8_t size) const override
  {
    char a[12];
    switch (_state)
    {
      case SensorState::STATE_ERROR:
        strncpy(out, "FAULT: ", size); out[size - 1] = '\0';
        if (_fault) { const uint8_t l = strlen(out); Wjs::copyFlash(out + l, size - l, _fault); }
        return;
      case SensorState::STATE_WARMING_UP:
        Wjs::fmtDuration(a, sizeof(a), getWarmupRemainingMs());
        snprintf(out, size, "Preheat left %s", a); return;
      default:
        if (!_haveFrame) snprintf(out, size, "UART: no frame yet");
        else snprintf(out, size, "UART OK (%lus ago)  bad frames %u", (unsigned long)((millis() - _lastFrameMs) / 1000UL), _bad);
        return;
    }
  }

  // ---------------------------------------------------------------- adjustable constants
  uint8_t paramCount() const override { return 3; }
  bool paramInfo(uint8_t i, ParamInfo& p) const override
  {
    switch (i)
    {
      case 0: p = { F("Zero offset"), F("ppm"), -5000.0f, 5000.0f, 0 }; break;
      case 1: p = { F("Span factor"), F(""),     0.5f,    2.0f,    4 }; break;
      case 2: p = { F("Smoothing"),   F(""),     0.02f,   1.0f,    2 }; break;
      default: return false;
    }
    return true;
  }
  float getParam(uint8_t i) const override { return i == 0 ? _zero : (i == 1 ? _span : _alpha); }
  bool setParam(uint8_t i, float v) override
  {
    ParamInfo p; if (!paramInfo(i, p)) return false;
    v = Wjs::clampf(v, p.minValue, p.maxValue);
    if (i == 0) _zero = v; else if (i == 1) _span = v; else _alpha = v;
    return true;
  }
  void resetParamDefaults() override { _zero = 0.0f; _span = 1.0f; _alpha = 0.3f; }
  bool supportsVolPercent() const override { return true; }
  uint8_t ppmDecimals() const override { return 0; }

  void printDashboard(Print& out) const override
  {
    out.print(F("[")); out.print(Wjs::stateName(_state)); out.print(F(" IR-CO2 | ")); Wjs::printHM(out, getTimeElapsed());
    out.print(F("] CO2=")); Wjs::printNum(out, getLivePPM(), 0); out.print(F("ppm"));
    if (_state == SensorState::STATE_WARMING_UP) { out.print(F(" preheat-left=")); Wjs::printHM(out, getWarmupRemainingMs()); }
    if (_state == SensorState::STATE_ERROR && _fault) { out.print(F(" FAULT: ")); out.print(_fault); }
    out.println();
  }

private:
  static constexpr uint32_t THREE_MIN_MS      {180000UL};
  static constexpr uint32_t POLL_MS           {1000UL};
  static constexpr uint32_t REPLY_TIMEOUT_MS  {6000UL};
  static constexpr uint16_t MAX_RAW           {60000};    // sensor range 50000; anything above is garbage

  HardwareSerial& _serial;
  uint32_t _warmupMs;
  SensorState _state{SensorState::STATE_OFF};
  const __FlashStringHelper* _fault{nullptr};

  float    _ppm{0.0f}, _alarmPPM{0.0f}, _zero{0.0f}, _span{1.0f}, _alpha{0.3f};
  float current() const { const float v = (_ppm - _zero) * _span; return v < 0.0f ? 0.0f : v; }   // raw smoothed -> trimmed
  uint32_t _startMs{0}, _lastPollMs{0}, _lastFrameMs{0};
  uint8_t  _buf[9];
  uint8_t  _idx{0}, _bad{0};
  bool     _haveFrame{false};

  /** Send the poll command when due and parse whatever arrived. @return true if a valid frame was decoded. */
  bool service(uint32_t now)
  {
    static const uint8_t CMD[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    if (now - _lastPollMs >= POLL_MS) { _lastPollMs = now; _serial.write(CMD, sizeof(CMD)); }

    bool got = false;
    while (_serial.available() > 0)
      if (feed(uint8_t(_serial.read()), now)) got = true;
    return got;
  }

  bool feed(uint8_t ch, uint32_t now)
  {
    if (_idx == 0) { if (ch == 0xFF) _buf[_idx++] = ch; return false; }
    if (_idx == 1)
    {
      if (ch == 0x86) _buf[_idx++] = ch;
      else if (ch != 0xFF) _idx = 0;                  // 0xFF again: treat it as the new frame start
      return false;
    }
    _buf[_idx++] = ch;
    if (_idx < sizeof(_buf)) return false;
    _idx = 0;

    uint8_t sum = 0;
    for (uint8_t i = 1; i < 8; ++i) sum += _buf[i];
    const uint8_t expect = uint8_t(uint8_t(~sum) + 1u);        // 256 - sum
    const uint16_t raw = uint16_t(_buf[2]) * 256U + _buf[3];
    if (expect != _buf[8] || raw > MAX_RAW) { if (_bad < 255) ++_bad; return false; }

    _bad = 0;
    _ppm = _haveFrame ? _ppm + _alpha * (float(raw) - _ppm) : float(raw);
    _haveFrame = true;
    _lastFrameMs = now;
    return true;
  }
};

#endif // WJS_IR_CO2_H
