#ifndef WJS_H2S_H
#define WJS_H2S_H

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_MultiGasSensor.h"
#include "WjsDTypes.h"

/*
  DFRobot Gravity calibrated electrochemical H2S sensor (SEN0467), I2C mode.
  Pipeline: I2C -> raw ppm + board temp -> (library temp compensation) -> state machine

  ADJUSTABLE CONSTANTS (paramCount() = 3), edited from the Nextion settings page:
    0 Zero offset : ppm subtracted from the module's reading (software zero trim)
    1 Span factor : multiplier applied after the zero offset (software span trim, 1.0000 = off)
    2 Lib temp comp: 0 = off (default), 1 = DFRobot library temperature compensation (see note below)
    final ppm = max(0, (raw - zero) * span)

  Facts verified against the DFRobot_MultiGasSensor library source (v2.x):
  * Default I2C address 0x74. DIP A1/A0: 00=0x74 01=0x75 10=0x76 11=0x77. DIP "SEL": 0 = I2C, 1 = UART.
    After changing SEL or the address the sensor must be power-cycled.
  * begin() only pings the address (ACK).
  * readGasConcentrationPPM()/readTempC() NEVER return NaN. A failed read gives 0.0 ppm, and an all-zero
    reply even passes the library checksum and decodes as -273.15 C. So link health is judged from the
    temperature plausibility, not from isnan().
  * setTempCompensation(ON) caches the board temperature ONCE (global _temp). If that cached value is outside
    -20..60 C the H2S branch returns 0.0 ppm for every later read.
  * The library's H2S compensation formula for T > 20 C is  ppm / (0.015*T - 0.3):  x13 at 25 C, x6.7 at 30 C and
    >x100 just above 20 C (and a x0.98 -> x133 jump at 20 C). That cannot be physical, so this driver leaves the
    library compensation OFF by default (the DFRobot wiki examples also use OFF). Enable it only after checking
    against a known H2S concentration: setLibraryTempCompensation(true).
  * Wiki: >5 min preheat after power-on; >24 h if the module was unused for a long time.

  References
  [1] DFRobot SEN0467 wiki + github.com/DFRobot/DFRobot_MultiGasSensor
*/

class WjsH2S : public IGasSensor {
public:
  enum : int { PPM_NOT_READY = -2, PPM_FAULT = -3 };

  /**
   * @param i2cAddress     0x74..0x77 (see DIP switches).
   * @param inactiveHours  Hours the SENSOR was unpowered (not the Arduino!). <1 -> 15 min, otherwise 24 h.
   * @param runInitialBake false -> only the 5 min minimum.
   */
  WjsH2S(uint8_t i2cAddress = 0x74, uint16_t inactiveHours = 0, bool runInitialBake = true)
    : _i2cAddr{i2cAddress}
    , _gas{ &Wire, i2cAddress }
    , _warmupMs{warmupFor(runInitialBake, inactiveHours)}
  {}

  /** Non-blocking. Returns false when the module does not ACK (driver keeps retrying in update). */
  bool begin()
  {
    const uint32_t now = millis();
    _startMs = _lastPollMs = _lastRetryMs = _lastTempRefreshMs = now;
    _fail = 0; _stage = 0; _linkOk = false; _fault = nullptr;
    _state = SensorState::STATE_WARMING_UP;
    if (!startInit(now)) { enterError(F("no I2C ACK")); return false; }
    return true;
  }

  SensorState updateAndGetState() override
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();

    // In ERROR the driver re-runs the init sequence; the state only leaves ERROR after a VERIFIED poll
    // (a bare I2C ACK is not enough - a half-dead module also ACKs).
    if (_state == SensorState::STATE_ERROR && _stage == 0 && now - _lastRetryMs >= RETRY_MS)
    {
      _lastRetryMs = now;
      if (startInit(now)) _fail = 0;
    }
    if (_stage == 1 && now - _initMs >= CONFIG_DELAY_MS) enableCompensation(now);
    if (_stage == 2 && now - _lastPollMs >= POLL_MS)     { _lastPollMs = now; poll(now); }
    if (_state == SensorState::STATE_ERROR) return _state;

    switch (_state)
    {
      case SensorState::STATE_WARMING_UP:
        if (now - _startMs >= _warmupMs) _state = SensorState::STATE_READY;
        break;
      case SensorState::STATE_CALIBRATING:
        _state = SensorState::STATE_READY;
        break;
      case SensorState::STATE_READY:
        if (_alarmPPM > 0.0f && _linkOk && _lastPPM >= _alarmPPM) _state = SensorState::STATE_ALARM;
        break;
      case SensorState::STATE_ALARM:
        if (_alarmPPM <= 0.0f || _lastPPM < 0.9f * _alarmPPM) _state = SensorState::STATE_READY;
        break;
      default: break;
    }
    return _state;
  }

  void setAlarmThreshold(float ppm) override { _alarmPPM = ppm; }

  /** Use the library's own temperature compensation (default false, see header note). Call before begin(). */
  void setLibraryTempCompensation(bool on)
  {
    _useLibComp = on;
    if (_stage == 2) { _gas.setTempCompensation(on ? _gas.ON : _gas.OFF); _lastTempRefreshMs = millis(); }
  }

  /** Override the preheat length (bench testing only). Call after begin(). */
  void setWarmupMs(uint32_t ms) { _warmupMs = ms; }

  /** Legacy API: ppm, or PPM_NOT_READY / PPM_FAULT codes. Prefer getReading(). */
  float getPPM() const
  {
    if (_state == SensorState::STATE_ERROR) return float(PPM_FAULT);
    if (!Wjs::isMeasuring(_state))          return float(PPM_NOT_READY);
    return _lastPPM;
  }

  bool getReading(float& ppm) const override
  {
    if (!Wjs::isMeasuring(_state) || !_linkOk) return false;
    ppm = _lastPPM;
    return true;
  }

  /** Last reading even while warming up (not yet stable!). NAN if the link is down. */
  float getLivePPM() const { return _linkOk ? _lastPPM : NAN; }
  float getTemperature() const { return _linkOk ? _lastTemp : NAN; }
  bool  isLinkOk() const { return _linkOk; }

  bool isPresent() const
  {
    Wire.beginTransmission(_i2cAddr);
    return Wire.endTransmission() == 0;
  }

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
        snprintf(out, size, "Warm-up left %s", a); return;
      default:
        Wjs::fmtFloat(a, sizeof(a), getTemperature(), 1);
        snprintf(out, size, "Board %sC  I2C %s @0x%02X", a, _linkOk ? "OK" : "DOWN", _i2cAddr); return;
    }
  }

  // ---------------------------------------------------------------- adjustable constants
  uint8_t paramCount() const override { return 3; }
  bool paramInfo(uint8_t i, ParamInfo& p) const override
  {
    switch (i)
    {
      case 0: p = { F("Zero offset"),   F("ppm"), -50.0f, 50.0f, 2 }; break;
      case 1: p = { F("Span factor"),   F(""),     0.1f,  5.0f,  4 }; break;
      case 2: p = { F("Lib temp comp"), F("0/1"),  0.0f,  1.0f,  0 }; break;
      default: return false;
    }
    return true;
  }
  float getParam(uint8_t i) const override { return i == 0 ? _zero : (i == 1 ? _span : (_useLibComp ? 1.0f : 0.0f)); }
  bool setParam(uint8_t i, float v) override
  {
    ParamInfo p; if (!paramInfo(i, p)) return false;
    v = Wjs::clampf(v, p.minValue, p.maxValue);
    if (i == 0) _zero = v; else if (i == 1) _span = v; else setLibraryTempCompensation(v >= 0.5f);
    return true;
  }
  void resetParamDefaults() override { _zero = 0.0f; _span = 1.0f; setLibraryTempCompensation(false); }
  uint8_t ppmDecimals() const override { return 1; }

  void printDashboard(Print& out) const override
  {
    out.print(F("[")); out.print(Wjs::stateName(_state)); out.print(F(" H2S | ")); Wjs::printHM(out, getTimeElapsed());
    out.print(F("] Temp=")); Wjs::printNum(out, getTemperature(), 1);
    out.print(F("C H2S=")); Wjs::printNum(out, getLivePPM(), 2); out.print(F("ppm"));
    out.print(F(" I2C=")); out.print(_linkOk ? F("OK") : F("DOWN"));
    if (_state == SensorState::STATE_WARMING_UP) { out.print(F(" warmup-left=")); Wjs::printHM(out, getWarmupRemainingMs()); }
    if (_state == SensorState::STATE_ERROR && _fault) { out.print(F(" FAULT: ")); out.print(_fault); }
    out.println();
  }

private:
  static constexpr uint32_t TWENTY_FOUR_H_MS {86400000UL};
  static constexpr uint32_t FIFTEEN_MIN_MS   {900000UL};
  static constexpr uint32_t FIVE_MIN_MS      {300000UL};
  static constexpr uint32_t POLL_MS          {1000UL};
  static constexpr uint32_t RETRY_MS         {2000UL};
  static constexpr uint32_t CONFIG_DELAY_MS  {1000UL};    // wiki example waits 1 s after changeAcquireMode
  static constexpr uint32_t TEMP_REFRESH_MS  {60000UL};   // refresh library's cached compensation temperature
  static constexpr uint32_t REWARM_AFTER_MS  {60000UL};
  static constexpr uint8_t  FAIL_LIMIT       {3};

  const uint8_t _i2cAddr;
  DFRobot_GAS_I2C _gas;
  uint32_t _warmupMs;

  SensorState _state{SensorState::STATE_OFF};
  const __FlashStringHelper* _fault{nullptr};
  float    _lastPPM{0.0f}, _lastTemp{0.0f}, _alarmPPM{0.0f};
  uint32_t _startMs{0}, _lastPollMs{0}, _lastRetryMs{0}, _initMs{0}, _lastTempRefreshMs{0}, _errorSinceMs{0};
  uint8_t  _stage{0}, _fail{0};      // _stage: 0 = not configured, 1 = mode set (waiting), 2 = running
  bool     _linkOk{false}, _useLibComp{false};

  static uint32_t warmupFor(bool bake, uint16_t inactiveHours)
  {
    if (!bake) return FIVE_MIN_MS;
    if (inactiveHours < 1) return FIFTEEN_MIN_MS;
    return TWENTY_FOUR_H_MS;
  }

  float _zero{0.0f}, _span{1.0f};
  float adjust(float c) const { const float v = (c - _zero) * _span; return v < 0.0f ? 0.0f : v; }

  static bool tempPlausible(float t) { return t > -40.0f && t < 100.0f; }

  bool startInit(uint32_t now)
  {
    if (!_gas.begin()) return false;                 // ACK ping
    _gas.changeAcquireMode(_gas.PASSIVITY);          // controller requests data
    _stage = 1; _initMs = now;
    return true;
  }

  /** Finish configuration once the board reports a believable temperature (also proves the link works). */
  void enableCompensation(uint32_t now)
  {
    const float t = _gas.readTempC();
    if (tempPlausible(t))
    {
      _gas.setTempCompensation(_useLibComp ? _gas.ON : _gas.OFF);   // ON also caches the temperature in the library
      _stage = 2; _lastTempRefreshMs = now; _fail = 0;
      return;
    }
    _initMs = now - CONFIG_DELAY_MS + POLL_MS;       // try again in ~1 s
    if (++_fail >= FAIL_LIMIT) enterError(F("bad reply (temp)"));
  }

  void poll(uint32_t now)
  {
    bool ok = isPresent();
    float t = 0.0f, c = 0.0f;
    if (ok)
    {
      t = _gas.readTempC();
      c = _gas.readGasConcentrationPPM();
      ok = tempPlausible(t) && !isnan(c) && c > -1000.0f && c < 100000.0f;
    }
    if (ok)
    {
      _fail = 0; _linkOk = true;
      _lastTemp = t; _lastPPM = adjust(c);
      if (_state == SensorState::STATE_ERROR) recover(now);
      if (_useLibComp && now - _lastTempRefreshMs >= TEMP_REFRESH_MS) { _lastTempRefreshMs = now; _gas.setTempCompensation(_gas.ON); }
    }
    else
    {
      _linkOk = false;
      if (++_fail >= FAIL_LIMIT) enterError(isPresent() ? F("bad reply (zeros)") : F("no I2C ACK"));
    }
  }

  void enterError(const __FlashStringHelper* why)
  {
    if (_state != SensorState::STATE_ERROR) _errorSinceMs = millis();   // keep the original outage start while retrying
    _state = SensorState::STATE_ERROR; _fault = why; _linkOk = false;
    _lastRetryMs = millis(); _stage = 0;
  }

  void recover(uint32_t now)
  {
    _fault = nullptr;
    if (now - _errorSinceMs > REWARM_AFTER_MS)       // module may have lost power: >=5 min stabilisation
    {
      _startMs = now; _warmupMs = FIVE_MIN_MS; _state = SensorState::STATE_WARMING_UP;
    }
    else _state = (now - _startMs >= _warmupMs) ? SensorState::STATE_READY : SensorState::STATE_WARMING_UP;
  }
};

#endif // WJS_H2S_H
