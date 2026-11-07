#ifndef GAS_CHANNEL_H
#define GAS_CHANNEL_H

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include "WjsDTypes.h"
#include "WjsGP8600.h"

/*
  GasChannel = ONE gas as the user sees it: a sensor + its 4-20 mA output + the settings that connect them.

  The settings page of the screen shows ONE list of constants per channel:
      [ the sensor's own constants ... ]   <- supplied by the sensor class (paramCount/paramInfo/getParam/setParam)
      Alarm 0=off   alarm level (0 switches the alarm off)
      4mA value     concentration that gives 4 mA
      20mA value    concentration that gives 20 mA
      DAC c0, DAC k0  output calibration from the Excel sheet (steps = c0 * nominal + k0)
      Unit 0/1      0 = ppm, 1 = vol%   (only for sensors that can show vol%)
  Concentration-type values (alarm, 4 mA, 20 mA) are shown in the CURRENT unit; inside they are always ppm.

  Everything is saved in ONE EEPROM block per channel (SLOT_BYTES), with a checksum. A block that does not match
  (new firmware, never saved, damaged) is ignored and the built-in defaults are used.
*/

class GasChannel {
public:
  static constexpr uint8_t MAX_PARAMS = 16;      // = number of rows on the Nextion settings page
  static constexpr int     SLOT_BYTES = 80;      // EEPROM bytes reserved per channel

  /**
   * @param code            short name, also used in Nextion component names: "CO", "CO2", "H2S", "H2", "CH4"
   * @param title           long name for the screen
   * @param eepromAddr      first EEPROM byte of this channel's slot
   * @param defaultAlarmPpm alarm level used until the user changes it (0 = off)
   * @param defaultDacHiPpm concentration that gives 20 mA (4 mA is always 0 by default)
   * @param defaultUnit     unit shown until the user changes it
   */
  GasChannel(const char* code, const char* title, IGasSensor& sensor, WjsGP8600& dac, int eepromAddr,
             float defaultAlarmPpm, float defaultDacHiPpm, GasUnit defaultUnit = GasUnit::PPM)
    : _code{code}, _title{title}, _sensor{sensor}, _dac{dac}, _eeAddr{eepromAddr}
    , _alarmDefault{defaultAlarmPpm}, _dacHiDefault{defaultDacHiPpm}, _unitDefault{defaultUnit}
    , _c0Default{dac.getC0()}, _k0Default{dac.getK0()}
    , _alarmPpm{defaultAlarmPpm}, _dacHiPpm{defaultDacHiPpm}
  { _sensor.setUnit(defaultUnit); _sensor.setAlarmThreshold(defaultAlarmPpm); }

  const char* code()  const { return _code; }
  const char* title() const { return _title; }
  IGasSensor& sensor() { return _sensor; }
  const IGasSensor& sensor() const { return _sensor; }
  WjsGP8600&  dac()    { return _dac; }
  const WjsGP8600& dac() const { return _dac; }

  // ================================================================ live values
  /** Call regularly: caches the sensor reading. */
  void update() { _valid = _sensor.getReading(_ppm); }
  bool  valid() const { return _valid; }
  float ppm()   const { return _ppm; }

  /** Reading in the display unit as text ("12.3", "0.042") or "---" when not valid. buf >= 14 chars. */
  void valueText(char* buf, uint8_t size) const
  {
    if (!_valid) { strncpy(buf, "---", size); buf[size - 1] = '\0'; return; }
    Wjs::fmtFloat(buf, size, _sensor.ppmToDisplay(_ppm), _sensor.displayDecimals());
  }
  const __FlashStringHelper* unitLabel() const { return _sensor.unitLabel(); }

  /** Drives the 4-20 mA output. forcedMA >= 0 overrides everything (bench test). Not valid/bad range -> 3.6 mA fault level. */
  void driveOutput(double forcedMA = -1.0)
  {
    if (forcedMA >= 0.0)              _dac.setmAOutput(forcedMA, false);
    else if (_valid && configOk())    _dac.setProcessValue(_ppm, _dacLoPpm, _dacHiPpm);
    else                              _dac.setFaultOutput();
  }
  /** The 20 mA value must be larger than the 4 mA value. */
  bool configOk() const { return _dacHiPpm > _dacLoPpm; }

  // ================================================================ constants (what the settings page edits)
  uint8_t paramCount() const { return uint8_t(_sensor.paramCount() + 5 + (_sensor.supportsVolPercent() ? 1 : 0)); }

  bool paramInfo(uint8_t i, ParamInfo& p) const
  {
    const uint8_t s = _sensor.paramCount();
    if (i < s) return _sensor.paramInfo(i, p);
    const float maxDisp = _sensor.ppmToDisplay(1000000.0f);
    const uint8_t d = _sensor.displayDecimals();
    switch (i - s)
    {
      case 0: p = { F("Alarm 0=off"), _sensor.unitLabel(), 0.0f, maxDisp, d }; return true;
      case 1: p = { F("4mA value"),   _sensor.unitLabel(), 0.0f, maxDisp, d }; return true;
      case 2: p = { F("20mA value"),  _sensor.unitLabel(), 0.0f, maxDisp, d }; return true;
      case 3: p = { F("DAC c0"),      F(""),               0.5f, 1.5f, 6 };    return true;
      case 4: p = { F("DAC k0"),      F("steps"),         -2000.0f, 2000.0f, 3 }; return true;
      case 5: if (!_sensor.supportsVolPercent()) return false;
              p = { F("Unit 0/1"),    _sensor.unitLabel(), 0.0f, 1.0f, 0 };   return true;
    }
    return false;
  }

  float getParam(uint8_t i) const
  {
    const uint8_t s = _sensor.paramCount();
    if (i < s) return _sensor.getParam(i);
    switch (i - s)
    {
      case 0: return _sensor.ppmToDisplay(_alarmPpm);
      case 1: return _sensor.ppmToDisplay(_dacLoPpm);
      case 2: return _sensor.ppmToDisplay(_dacHiPpm);
      case 3: return float(_dac.getC0());
      case 4: return float(_dac.getK0());
      case 5: return _sensor.getUnit() == GasUnit::VOL_PERCENT ? 1.0f : 0.0f;
    }
    return 0.0f;
  }

  /** Applies one constant (clamped to its range). Concentration values are interpreted in the CURRENT unit. */
  bool setParam(uint8_t i, float v)
  {
    ParamInfo p; if (!paramInfo(i, p)) return false;
    v = Wjs::clampf(v, p.minValue, p.maxValue);
    const uint8_t s = _sensor.paramCount();
    if (i < s) return _sensor.setParam(i, v);
    switch (i - s)
    {
      case 0: _alarmPpm = _sensor.displayToPpm(v); _sensor.setAlarmThreshold(_alarmPpm); break;
      case 1: _dacLoPpm = _sensor.displayToPpm(v); break;
      case 2: _dacHiPpm = _sensor.displayToPpm(v); break;
      case 3: _dac.setCalibration(v, _dac.getK0()); break;
      case 4: _dac.setCalibration(_dac.getC0(), v); break;
      case 5: _sensor.setUnit(v >= 0.5f ? GasUnit::VOL_PERCENT : GasUnit::PPM); break;
    }
    return true;
  }

  /** Built-in defaults for everything (a MEMS R0 is kept). */
  void resetDefaults()
  {
    _sensor.resetParamDefaults();
    _sensor.setUnit(_unitDefault);
    _alarmPpm = _alarmDefault; _dacLoPpm = 0.0f; _dacHiPpm = _dacHiDefault;
    _dac.setCalibration(_c0Default, _k0Default);
    _sensor.setAlarmThreshold(_alarmPpm);
  }

  // ================================================================ EEPROM
  void save() const
  {
    Record r; memset(&r, 0, sizeof(r));
    const uint8_t n = paramCount();
    r.magic = MAGIC; r.count = n; r.unit = (_sensor.getUnit() == GasUnit::VOL_PERCENT) ? 1 : 0;
    for (uint8_t i = 0; i < n && i < MAX_PARAMS; ++i) r.v[i] = getParam(i);
    r.crc = Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc));
    EEPROM.put(_eeAddr, r);                         // put() only rewrites bytes that changed (saves EEPROM wear)
  }

  /** Loads and APPLIES the saved constants. Returns false (defaults stay) when nothing valid is stored. */
  bool load()
  {
    Record r; EEPROM.get(_eeAddr, r);
    if (r.magic != MAGIC || r.count != paramCount() || r.count > MAX_PARAMS) return false;
    if (r.crc != Wjs::crc8(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc))) return false;
    _sensor.setUnit(r.unit ? GasUnit::VOL_PERCENT : GasUnit::PPM);   // unit FIRST: saved values are in that unit
    for (uint8_t i = 0; i < r.count; ++i) if (!isnan(r.v[i])) setParam(i, r.v[i]);
    return true;
  }

  /** Forget the stored block (defaults and a full MEMS preheat after the next restart). */
  void eraseStored() const { const uint16_t blank = 0xFFFF; EEPROM.put(_eeAddr, blank); }

private:
  static constexpr uint16_t MAGIC = 0x5C3D;
  struct Record { uint16_t magic; uint8_t count; uint8_t unit; float v[MAX_PARAMS]; uint8_t crc; };

  const char* _code; const char* _title;
  IGasSensor& _sensor; WjsGP8600& _dac;
  const int   _eeAddr;
  const float _alarmDefault, _dacHiDefault;
  const GasUnit _unitDefault;
  const double _c0Default, _k0Default;
  float _alarmPpm, _dacLoPpm{0.0f}, _dacHiPpm;
  float _ppm{0.0f};
  bool  _valid{false};
};

#endif
