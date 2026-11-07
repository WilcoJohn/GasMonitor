#ifndef WJS_MEMS_H
#define WJS_MEMS_H

#include <Arduino.h>
#include <math.h>
#include "WjsDTypes.h"
#include "WjsMemsData.h"

/*
  Winsen MEMS MOX gas sensor (CH4, H2S, CO, H2) on a DFRobot Fermion resistor-divider board.
  Pipeline: ADC -> Vout -> Rs -> Rs/K(T,RH) -> Rs/R0 -> ppm = a * (Rs/R0)^b

  ADJUSTABLE CONSTANTS (paramCount() = 9) - these are what the Nextion settings page edits:
    0 R0 clean air  : sensor resistance in clean air at 20 C (kOhm). Set by ZERO calibration or typed in.
    1 Supply Vc     : voltage that powers the sensor board's divider (measure it with a multimeter!)
    2 Load RL       : load resistor on the board (4.7 kOhm on the Fermion boards)
    3 Curve a low   \  ppm = a * (Rs/R0)^b   for Rs/R0 >= Split ratio   (low concentrations)
    4 Curve b low   /
    5 Curve a high  \  same formula for Rs/R0 <  Split ratio            (high concentrations)
    6 Curve b high  /
    7 Split ratio   : where the two curves meet (0 = use the "low" curve only)
    8 No-gas ratio  : Rs/R0 at/above this is reported as 0 ppm

  Behaviour
  - ADC is sampled ONCE per SAMPLE_MS and cached (also during warm-up, so a missing sensor is noticed at once).
  - Floating-pin probe -> ERROR instead of garbage readings. Retries automatically.
  - If R0 is already known when begin() runs (loaded from EEPROM by the app) only a "quick" 15 min warm-up is
    used; with no R0 the full preheat (24-168 h, from the profile) runs first, then a zero calibration.
  - The driver never writes EEPROM itself. When it changes a constant (calibration finished) it raises a flag
    (consumeParamsChanged) and the application saves everything.

  References
  [1] Winsen GM-402B / GM-602B / GM-702B / GMV-2021B manuals (preheat table, curves, T/RH figure)
  [2] ATmega2560 datasheet: ADC result = Vin * 1024 / Vref
  [3] DFRobot SEN0564 wiki: RL = 4.7 kOhm, qualitative sensor (not for personal-safety use)
*/

class WjsMEMS : public IGasSensor {
public:
  /**
   * @param pin             Analog input pin constant (A0...).
   * @param profile         WjsMemsData::CH4 / H2S / CO / H2
   * @param storageMonths   Months the sensor was stored unpowered (0 / 1-6 / >6): picks the full preheat time.
   * @param vRef            ADC reference voltage.
   * @param adcSteps        ADC steps (1024 on AVR).
   * @param vSupply         Voltage powering the sensor board (0 = same as vRef).
   * @param allowVolPercent true for H2 and CH4 (screen may switch to vol%).
   * @param ppmDec          decimals shown in ppm.
   */
  WjsMEMS(uint8_t pin, const MemsProfile& profile, int storageMonths,
          float vRef = 5.0f, float adcSteps = 1024.0f, float vSupply = 0.0f,
          bool allowVolPercent = false, uint8_t ppmDec = 1)
    : m_pin{pin}
    , m_profile{&profile}
    , m_vRef{vRef}
    , m_adcSteps{adcSteps}
    , m_vSupplyDefault{vSupply > 0.0f ? vSupply : vRef}
    , m_fullWarmupMs{fullWarmup(profile, storageMonths)}
    , m_allowVol{allowVolPercent}
    , m_ppmDec{ppmDec}
    , m_vSupply{vSupply > 0.0f ? vSupply : vRef}
    , m_rLoad{profile.rLoad}
    , m_aLow{profile.aLow},   m_bLow{profile.bLow}
    , m_aHigh{profile.aHigh}, m_bHigh{profile.bHigh}
    , m_split{profile.splitRatio}
    , m_noGas{profile.noGasRatio}
    , m_tempC{profile.comp ? profile.comp->refT  : 20.0f}
    , m_rh   {profile.comp ? profile.comp->refRH : 55.0f}
  {}

  /** Call once in setup(), AFTER the stored constants were loaded (setParam). */
  void begin()
  {
    pinMode(m_pin, INPUT);
    const uint32_t now = millis();
    m_startMs = m_lastSampleMs = m_lastProbeMs = now;
    m_vSeeded = false; m_bad = 0; m_fault = nullptr;
    m_quick = (m_r0 > 0.0f);                              // R0 known -> quick warm-up
    m_warmupMs = m_quick ? QUICK_WARMUP_MS : m_fullWarmupMs;
    m_state = SensorState::STATE_WARMING_UP;
    if (Wjs::analogPinFloating(m_pin, m_adcSteps)) enterError(F("no sensor (pin floating)"));
  }

  /** Non-blocking. Call every loop(). Does real work once per SAMPLE_MS. */
  SensorState updateAndGetState() override
  {
    if (m_state == SensorState::STATE_OFF) return m_state;
    const uint32_t now = millis();

    if (m_state == SensorState::STATE_ERROR) { retry(now); return m_state; }
    if (m_vSeeded && now - m_lastSampleMs < SAMPLE_MS) return m_state;
    m_lastSampleMs = now;

    const float raw = readVoltage();
    if (!voltageValid(raw))
    {
      if (++m_bad >= BAD_LIMIT) enterError(F("voltage out of range"));
      return m_state;
    }
    m_bad = 0;
    m_v = m_vSeeded ? m_v + EMA_ALPHA * (raw - m_v) : raw;
    m_vSeeded = true;

    if (now - m_lastProbeMs >= PROBE_MS)
    {
      m_lastProbeMs = now;
      if (Wjs::analogPinFloating(m_pin, m_adcSteps)) { enterError(F("sensor disconnected")); return m_state; }
    }

    switch (m_state)
    {
      case SensorState::STATE_WARMING_UP:
        if (now - m_startMs >= m_warmupMs)
        {
          if (m_r0 > 0.0f) { m_lastBaseMs = now; m_state = SensorState::STATE_READY; }
          else             startCalibration();
        }
        break;
      case SensorState::STATE_CALIBRATING: calibrationStep(now); break;
      case SensorState::STATE_READY:
      case SensorState::STATE_ALARM:       measureStep(now);     break;
      default: break;
    }
    return m_state;
  }

  /** Ambient temp (C) / RH (%) from the DHT. Only call with valid readings. */
  void setEnvironment(float tempC, float rhPct) { m_tempC = tempC; m_rh = rhPct; }

  void setAlarmThreshold(float ppm) override { m_alarmPPM = ppm; }
  /** Override the current warm-up length (bench testing only). Call after begin(). */
  void setWarmupMs(uint32_t ms) { m_warmupMs = ms; }

  /** Start zero calibration in CLEAN AIR (10 s). The new R0 is reported through consumeParamsChanged(). */
  bool recalibrate() override
  {
    if (!Wjs::isMeasuring(m_state)) return false;
    startCalibration();
    return true;
  }

  // ---------------------------------------------------------------- adjustable constants
  uint8_t paramCount() const override { return 9; }

  bool paramInfo(uint8_t i, ParamInfo& p) const override
  {
    switch (i)
    {
      case 0: p = { F("R0 clean air"), F("kOhm"), R0_MIN, R0_MAX, 3 }; break;
      case 1: p = { F("Supply Vc"),    F("V"),    1.0f, 30.0f, 2 };    break;
      case 2: p = { F("Load RL"),      F("kOhm"), 0.1f, 1000.0f, 2 };  break;
      case 3: p = { F("Curve a low"),  F(""),     0.000001f, 1000000.0f, 5 }; break;
      case 4: p = { F("Curve b low"),  F(""),    -30.0f, 30.0f, 5 };   break;
      case 5: p = { F("Curve a high"), F(""),     0.000001f, 1000000.0f, 5 }; break;
      case 6: p = { F("Curve b high"), F(""),    -30.0f, 30.0f, 5 };   break;
      case 7: p = { F("Split ratio"),  F(""),     0.0f, 2.0f, 3 };     break;
      case 8: p = { F("No-gas ratio"), F(""),     0.5f, 2.0f, 3 };     break;
      default: return false;
    }
    return true;
  }

  float getParam(uint8_t i) const override
  {
    switch (i)
    {
      case 0: return m_r0;     case 1: return m_vSupply; case 2: return m_rLoad;
      case 3: return m_aLow;   case 4: return m_bLow;    case 5: return m_aHigh;
      case 6: return m_bHigh;  case 7: return m_split;   case 8: return m_noGas;
    }
    return 0.0f;
  }

  bool setParam(uint8_t i, float v) override
  {
    ParamInfo p; if (!paramInfo(i, p)) return false;
    v = Wjs::clampf(v, p.minValue, p.maxValue);
    switch (i)
    {
      case 0: m_r0 = m_r0Cal = v; break;
      case 1: m_vSupply = v; m_vSeeded = false; break;      // restart the filter with the new divider voltage
      case 2: m_rLoad = v; break;
      case 3: m_aLow = v; break;   case 4: m_bLow = v; break;
      case 5: m_aHigh = v; break;  case 6: m_bHigh = v; break;
      case 7: m_split = v; break;  case 8: m_noGas = v; break;
    }
    return true;
  }

  void resetParamDefaults() override
  {
    m_vSupply = m_vSupplyDefault; m_rLoad = m_profile->rLoad;
    m_aLow = m_profile->aLow;   m_bLow = m_profile->bLow;
    m_aHigh = m_profile->aHigh; m_bHigh = m_profile->bHigh;
    m_split = m_profile->splitRatio; m_noGas = m_profile->noGasRatio;
    m_vSeeded = false;                                    // R0 is deliberately kept
  }

  bool consumeParamsChanged() override { const bool d = m_dirty; m_dirty = false; return d; }

  bool    supportsVolPercent() const override { return m_allowVol; }
  uint8_t ppmDecimals() const override { return m_ppmDec; }

  // ---------------------------------------------------------------- measurements (cached, no ADC access)
  float getPinVoltage() const    { return m_vSeeded ? m_v : NAN; }
  float getRs() const            { return m_vSeeded ? rsFromVoltage(m_v) : NAN; }
  float getRsCompensated() const { return compensate(getRs()); }
  float getRatio() const         { return ratioFromRs(getRsCompensated()); }
  float readPPM() const          { return ppmFromRatio(getRatio()); }
  float getR0() const            { return m_r0; }

  bool getReading(float& ppm) const override
  {
    if (!Wjs::isMeasuring(m_state)) return false;
    const float p = readPPM();
    if (isnan(p)) return false;
    ppm = p;
    return true;
  }

  SensorState getState() const override { return m_state; }
  const __FlashStringHelper* getFaultText() const override { return (m_state == SensorState::STATE_ERROR) ? m_fault : nullptr; }
  uint32_t getWarmupRemainingMs() const override
  {
    return (m_state == SensorState::STATE_WARMING_UP) ? Wjs::remainingMs(m_startMs, m_warmupMs) : 0UL;
  }
  uint8_t getCalibrationProgress() const { return uint8_t((uint16_t(m_calN) * 100U) / CAL_SAMPLES); }
  bool    isQuickWarmup() const { return m_quick; }
  const char* getName() const { return m_profile->name; }
  unsigned long getTimeElapsed() const { return (m_state == SensorState::STATE_OFF) ? 0UL : (millis() - m_startMs); }

  void getStatusLine(char* out, uint8_t size) const override
  {
    char a[12], b[12], c[12];
    switch (m_state)
    {
      case SensorState::STATE_ERROR:
        strncpy(out, "FAULT: ", size); out[size - 1] = '\0';
        if (m_fault) { const uint8_t l = strlen(out); Wjs::copyFlash(out + l, size - l, m_fault); }
        return;
      case SensorState::STATE_WARMING_UP:
        Wjs::fmtDuration(a, sizeof(a), getWarmupRemainingMs());
        snprintf(out, size, "Warm-up left %s (%s)", a, m_quick ? "quick" : "full"); return;
      case SensorState::STATE_CALIBRATING:
        snprintf(out, size, "Zeroing %u%% - keep CLEAN AIR", getCalibrationProgress()); return;
      default:
        Wjs::fmtFloat(a, sizeof(a), getPinVoltage(), 3);
        Wjs::fmtFloat(b, sizeof(b), getRsCompensated(), 2);
        Wjs::fmtFloat(c, sizeof(c), getRatio(), 3);
        snprintf(out, size, "V %s  Rs %sk  ratio %s", a, b, c); return;
    }
  }

  void printDashboard(Print& out) const override
  {
    out.print(F("[")); out.print(Wjs::stateName(m_state)); out.print(' ');
    out.print(m_profile->name); out.print(F(" | ")); Wjs::printHM(out, getTimeElapsed()); out.print(F("] V="));
    Wjs::printNum(out, getPinVoltage(), 3);       out.print(F(" Rs="));
    Wjs::printNum(out, getRsCompensated(), 2);    out.print(F("k R0="));
    Wjs::printNum(out, m_r0, 2);                  out.print(F("k ratio="));
    Wjs::printNum(out, getRatio(), 3);            out.print(F(" ppm="));
    Wjs::printNum(out, readPPM(), 1);
    if (m_state == SensorState::STATE_WARMING_UP) { out.print(F(" warmup-left=")); Wjs::printHM(out, getWarmupRemainingMs()); out.print(m_quick ? F(" (quick)") : F(" (full)")); }
    if (m_state == SensorState::STATE_CALIBRATING) { out.print(F(" calib=")); out.print(getCalibrationProgress()); out.print('%'); }
    if (m_state == SensorState::STATE_ERROR && m_fault) { out.print(F(" FAULT: ")); out.print(m_fault); }
    out.println();
  }

private:
  static constexpr uint8_t  CAL_SAMPLES   {40};          // x SAMPLE_MS = 10 s of clean air
  static constexpr uint16_t SAMPLE_MS     {250};
  static constexpr uint8_t  BAD_LIMIT     {5};
  static constexpr float    EMA_ALPHA     {0.25f};
  static constexpr float    V_MARGIN      {0.05f};
  static constexpr uint32_t PROBE_MS      {30000UL};
  static constexpr uint32_t RETRY_MS      {2000UL};
  static constexpr uint32_t REWARM_AFTER_MS{60000UL};
  static constexpr uint32_t QUICK_WARMUP_MS{900000UL};   // 15 min when R0 is already known
  static constexpr uint32_t BASE_PERIOD_MS{60000UL};
  static constexpr float    BASE_ALPHA    {0.01f};
  static constexpr float    BASE_MIN_RATIO{0.95f};       // slow drift tracking only in clean air
  static constexpr float    BASE_MAX_DRIFT{0.30f};
  static constexpr uint32_t MS_PER_HOUR   {3600000UL};
  static constexpr float    R0_MIN        {0.2f};
  static constexpr float    R0_MAX        {1000.0f};

  const uint8_t       m_pin;
  const MemsProfile*  m_profile;
  const float         m_vRef, m_adcSteps, m_vSupplyDefault;
  const uint32_t      m_fullWarmupMs;
  const bool          m_allowVol;
  const uint8_t       m_ppmDec;
  float               m_vSupply, m_rLoad;
  float               m_aLow, m_bLow, m_aHigh, m_bHigh, m_split, m_noGas;
  float               m_tempC, m_rh;

  SensorState m_state{ SensorState::STATE_OFF };
  const __FlashStringHelper* m_fault{nullptr};
  uint32_t    m_warmupMs{0}, m_startMs{0}, m_lastSampleMs{0}, m_lastBaseMs{0}, m_lastProbeMs{0}, m_errorSinceMs{0};
  float       m_v{0.0f}, m_r0{0.0f}, m_r0Cal{0.0f}, m_alarmPPM{0.0f}, m_calSum{0.0f};
  uint8_t     m_calN{0}, m_bad{0};
  bool        m_vSeeded{false}, m_quick{false}, m_dirty{false};

  static uint32_t fullWarmup(const MemsProfile& p, int months)
  {
    const uint8_t i = (months < 1) ? 0 : ((months <= 6) ? 1 : 2);
    return uint32_t(p.warmupH[i]) * MS_PER_HOUR;
  }

  /** Dummy read (mux settle) + 8-sample average -> volts. */
  float readVoltage() const
  {
    analogRead(m_pin);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) sum += analogRead(m_pin);
    return (sum / 8.0f) * m_vRef / m_adcSteps;
  }

  float hiLimit() const { return (m_vSupply < m_vRef ? m_vSupply : m_vRef) - V_MARGIN; }
  bool  voltageValid(float v) const { return v > V_MARGIN && v < hiLimit(); }

  float rsFromVoltage(float v) const
  {
    if (!voltageValid(v)) return NAN;
    return m_rLoad * (m_vSupply - v) / v;                       // Rs = RL*(Vc-Vout)/Vout
  }

  static void locate(const float* ax, uint8_t n, float x, uint8_t& i, float& t)
  {
    if (x <= ax[0])     { i = 0;     t = 0.0f; return; }
    if (x >= ax[n - 1]) { i = n - 2; t = 1.0f; return; }
    i = 0;
    while (x > ax[i + 1]) ++i;
    t = (x - ax[i]) / (ax[i + 1] - ax[i]);
  }

  static float lookupK(const MemsCompTable& c, float tC, float rh)
  {
    uint8_t i, j; float t, u;
    locate(c.tempC, c.nT,  tC, i, t);
    locate(c.rhPct, c.nRH, rh, j, u);
    const float k00 = c.k[i * c.nRH + j],       k01 = c.k[i * c.nRH + j + 1];
    const float k10 = c.k[(i + 1) * c.nRH + j], k11 = c.k[(i + 1) * c.nRH + j + 1];
    return (1 - t) * ((1 - u) * k00 + u * k01) + t * ((1 - u) * k10 + u * k11);
  }

  /** Rs at the reference T/RH = Rs * K(ref) / K(T,RH). */
  float compensate(float rs) const
  {
    const MemsCompTable* c = m_profile->comp;
    if (isnan(rs) || !c) return rs;
    const float kRef = lookupK(*c, c->refT, c->refRH);
    const float k    = lookupK(*c, m_tempC, m_rh);
    return (k > 0.0f && kRef > 0.0f) ? rs * kRef / k : rs;
  }

  float ratioFromRs(float rsComp) const { return (isnan(rsComp) || m_r0 <= 0.0f) ? NAN : rsComp / m_r0; }

  float ppmFromRatio(float ratio) const
  {
    if (isnan(ratio)) return NAN;
    if (ratio >= m_noGas) return 0.0f;
    const bool  low = ratio >= m_split;
    const float ppm = (low ? m_aLow : m_aHigh) * powf(ratio, low ? m_bLow : m_bHigh);
    return (ppm > m_profile->maxPPM) ? m_profile->maxPPM : ppm;
  }

  void enterError(const __FlashStringHelper* why)
  {
    m_state = SensorState::STATE_ERROR; m_fault = why;
    m_errorSinceMs = m_lastProbeMs = millis(); m_bad = 0;
  }

  void retry(uint32_t now)
  {
    if (now - m_lastProbeMs < RETRY_MS) return;
    m_lastProbeMs = now;
    if (Wjs::analogPinFloating(m_pin, m_adcSteps)) { m_fault = F("no sensor (pin floating)"); return; }
    if (!voltageValid(readVoltage()))              { m_fault = F("voltage out of range");     return; }
    m_fault = nullptr; m_vSeeded = false; m_bad = 0;
    if (now - m_errorSinceMs > REWARM_AFTER_MS)      // sensor may have lost power: stabilise again
    {
      m_quick = (m_r0 > 0.0f);
      m_warmupMs = m_quick ? QUICK_WARMUP_MS : m_fullWarmupMs;
      m_startMs = now; m_state = SensorState::STATE_WARMING_UP;
    }
    else if (now - m_startMs < m_warmupMs)           m_state = SensorState::STATE_WARMING_UP;
    else if (m_r0 > 0.0f)                            { m_lastBaseMs = now; m_state = SensorState::STATE_READY; }
    else                                             startCalibration();
  }

  void startCalibration() { m_calSum = 0.0f; m_calN = 0; m_state = SensorState::STATE_CALIBRATING; }

  void calibrationStep(uint32_t now)
  {
    const float rs = getRsCompensated();
    if (isnan(rs)) return;
    m_calSum += rs;
    if (++m_calN >= CAL_SAMPLES)
    {
      const float r0 = m_calSum / CAL_SAMPLES;
      if (r0 < R0_MIN || r0 > R0_MAX) { enterError(F("R0 implausible")); return; }
      m_r0 = m_r0Cal = r0;
      m_lastBaseMs = now;
      m_dirty = true;                                 // the app saves the new R0
      m_state = SensorState::STATE_READY;
    }
  }

  void measureStep(uint32_t now)
  {
    const float rs = getRsCompensated();
    if (isnan(rs) || m_r0 <= 0.0f) return;
    const float ratio = rs / m_r0;
    const float ppm   = ppmFromRatio(ratio);

    if (m_alarmPPM > 0.0f)
    {
      if (m_state == SensorState::STATE_READY && ppm >= m_alarmPPM)             m_state = SensorState::STATE_ALARM;
      else if (m_state == SensorState::STATE_ALARM && ppm < 0.9f * m_alarmPPM)  m_state = SensorState::STATE_READY;
    }
    else if (m_state == SensorState::STATE_ALARM) m_state = SensorState::STATE_READY;

    if (m_state == SensorState::STATE_READY && ratio >= BASE_MIN_RATIO && now - m_lastBaseMs >= BASE_PERIOD_MS)
    {
      m_lastBaseMs = now;
      const float r0 = m_r0 + BASE_ALPHA * (rs - m_r0);
      const float lo = m_r0Cal * (1.0f - BASE_MAX_DRIFT), hi = m_r0Cal * (1.0f + BASE_MAX_DRIFT);
      m_r0 = (r0 < lo) ? lo : (r0 > hi ? hi : r0);
    }
  }
};

#endif
