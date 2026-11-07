#ifndef WJS_GP8600_H
#define WJS_GP8600_H

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_GP8XXX.h>      // also needs the DFRobot_RTU library installed (wiki: dependency)
#include "WjsDTypes.h"

/*
  WjsGP8600: DFRobot GP8600 (DFR1229) 16-bit I2C DAC, used in its 0-20 mA range as a 4-20 mA transmitter.
  I2C mode: 0..65535 = 0..20 mA. Default address 0x58 (DIP A0/A1/A2 -> 0x58..0x5F).

  Calibration (Excel sheet "DAC-output-calibration"):
      steps_written = c0 * nominalSteps + k0        nominalSteps = mA / 20 * 65535
  Constructor takes c0 (slope) and k0 (intercept). Use c0 = 1, k0 = 0 to calibrate.

  Behaviour
  * Never blocks; begin() failing (module missing) is not fatal - the driver retries every 2 s.
  * While READY a presence watchdog runs every CHECK_MS; a vanished module -> ERROR.
  * After a reconnect the last requested output is re-applied automatically.
  * Output is refreshed every REFRESH_MS (also re-sends the range register in case the DAC browned out).
  * Faults are signalled BELOW 4 mA (default 3.6 mA, NAMUR NE43 style) so a PLC can tell "sensor fault /
    warming up" from "0 ppm" (= 4.00 mA). Change faultMA() if your receiver cannot handle <4 mA.
  * NOTE: on AVR "double" is only 32-bit float - plenty for 16-bit step values.

  References
  [1] wiki.dfrobot.com/dfr1229 (API: setDACOutRange(eOutputRange20MA), setDACOutData(uint16_t), begin() == 0 on success)
  [2] github.com/DFRobot/DFRobot_GP8XXX
*/

class WjsGP8600 {
public:
  static double faultMA() { return 3.6; }

  WjsGP8600(uint8_t address, double conv_slope = 1.0, double conv_intercept = 0.0)
    : _address{address}, _DAC{&Wire, address}, _slope{conv_slope}, _intercept{conv_intercept}
  {}

  bool begin()
  {
    const uint32_t now = millis();
    _lastRetryMs = _lastCheckMs = _lastWriteMs = now;
    if (tryInit()) { _state = SensorState::STATE_READY; _linkOk = true; return true; }
    _state = SensorState::STATE_ERROR; _linkOk = false;
    return false;
  }

  SensorState updateAndGetState()
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();

    if (_state == SensorState::STATE_ERROR)
    {
      if (now - _lastRetryMs >= RETRY_MS)
      {
        _lastRetryMs = now;
        if (tryInit()) { _state = SensorState::STATE_READY; _linkOk = true; _lastCheckMs = now; if (_hasOutput) writeSteps(now); }
      }
      return _state;
    }

    if (now - _lastCheckMs >= CHECK_MS)
    {
      _lastCheckMs = now;
      if (!isPresent()) { _state = SensorState::STATE_ERROR; _linkOk = false; _lastRetryMs = now; return _state; }
    }
    if (_hasOutput && now - _lastWriteMs >= REFRESH_MS)
    {
      _DAC.setDACOutRange(_DAC.eOutputRange20MA);
      writeSteps(now);
    }
    return _state;
  }

  /** Write nominal DAC steps (0..65535 = 0..20 mA); c0/k0 calibration is applied here. */
  bool setOutputValue(double nominalSteps)
  {
    double raw = (_slope * nominalSteps) + _intercept;
    if (raw < 0.0)     raw = 0.0;
    if (raw > 65535.0) raw = 65535.0;
    const uint16_t steps = static_cast<uint16_t>(raw + 0.5);
    const bool changed = !_hasOutput || steps != _lastRawSteps;
    _lastRawSteps = steps;
    _hasOutput = true;
    if (_state != SensorState::STATE_READY) return false;     // remembered, applied after recovery
    if (changed) writeSteps(millis());                        // unchanged value: periodic refresh handles it
    return true;
  }

  /** Output a current. limitTo4_20 = false allows 0..20 mA (calibration / fault levels). */
  bool setmAOutput(double target_mA, bool limitTo4_20 = true)
  {
    const double lo = limitTo4_20 ? 4.0 : 0.0;
    if (target_mA < lo)   target_mA = lo;
    if (target_mA > 20.0) target_mA = 20.0;
    _lastTargetMA = target_mA;
    return setOutputValue((target_mA / 20.0) * 65535.0);
  }

  /** Map a process value onto 4..20 mA (rangeLo -> 4 mA, rangeHi -> 20 mA), clamped. */
  bool setProcessValue(float value, float rangeLo, float rangeHi)
  {
    if (isnan(value) || rangeHi <= rangeLo) return setFaultOutput();
    double f = double(value - rangeLo) / double(rangeHi - rangeLo);
    if (f < 0.0) f = 0.0;
    if (f > 1.0) f = 1.0;
    return setmAOutput(4.0 + 16.0 * f);
  }

  /** Drive the fault level (below 4 mA). */
  bool setFaultOutput() { return setmAOutput(faultMA(), false); }

  /** Output calibration from the Excel sheet: steps written = c0 * nominalSteps + k0. */
  void   setCalibration(double c0, double k0) { _slope = c0; _intercept = k0; if (_hasOutput && _state == SensorState::STATE_READY) writeSteps(millis()); }
  double getC0() const { return _slope; }
  double getK0() const { return _intercept; }

  SensorState getState() const { return _state; }
  bool isLinkOk() const { return _linkOk; }
  uint16_t getLastRawSteps() const { return _lastRawSteps; }
  double getLastTargetMA() const { return _lastTargetMA; }
  uint8_t getAddress() const { return _address; }

  bool isPresent() const
  {
    Wire.beginTransmission(_address);
    return Wire.endTransmission() == 0;
  }

  void printDashboard(Print& out) const
  {
    out.print(F("[")); out.print(Wjs::stateName(_state));
    out.print(F(" DAC 0x")); out.print(_address, HEX);
    out.print(F("] mA=")); out.print(_lastTargetMA, 2);
    out.print(F(" steps=")); out.print(_lastRawSteps);
    out.print(F(" I2C=")); out.println(_linkOk ? F("OK") : F("NONE"));
  }

private:
  static constexpr uint32_t RETRY_MS   {2000UL};
  static constexpr uint32_t CHECK_MS   {2000UL};
  static constexpr uint32_t REFRESH_MS {10000UL};

  uint8_t _address;
  DFRobot_GP8600_I2C _DAC;
  double _slope, _intercept;

  SensorState _state{SensorState::STATE_OFF};
  uint16_t _lastRawSteps{0};
  double   _lastTargetMA{0.0};
  uint32_t _lastRetryMs{0}, _lastCheckMs{0}, _lastWriteMs{0};
  bool     _linkOk{false}, _hasOutput{false};

  void writeSteps(uint32_t now) { _DAC.setDACOutData(_lastRawSteps); _lastWriteMs = now; }

  bool tryInit()
  {
    if (!isPresent()) return false;
    if (_DAC.begin() != 0) return false;
    _DAC.setDACOutRange(_DAC.eOutputRange20MA);   // enforce 0-20 mA range
    return true;
  }
};

#endif // WJS_GP8600_H
