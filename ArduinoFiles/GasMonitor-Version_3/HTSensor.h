#ifndef WJS_HT_SENSOR_H
#define WJS_HT_SENSOR_H

#include <Arduino.h>
#include <SimpleDHT.h>
#include "WjsDTypes.h"

/*
  WjsHTSensor: DHT11 / DHT22 temperature + humidity wrapper (SimpleDHT library).
  Define WJS_HT_DHT22 before including this file for a DHT22 (default is DHT11).

  Changes vs the original
  * DHT11 only returned whole degrees through the old byte API; read2() gives floats (DHT22 -> 0.1 resolution).
  * Default pin no longer A0 (that is the H2 MEMS input).
  * Explicit validity: getReading() is false until a plausible reading exists and after STALE_MS without one,
    so a fault value (-99) can never leak into the gas compensation maths.
  * One read per interval (the old code read twice per cycle while in ERROR).
  * Plain "C" in text (a degree sign is not in the default Nextion font encoding).
  Note: SimpleDHT needs ~20-30 ms with tight timing for each read; keep the interval >= 2 s.
*/

#ifdef WJS_HT_DHT22
  typedef SimpleDHT22 WjsDhtDriver;
  #define WJS_HT_NAME "DHT22"
#else
  typedef SimpleDHT11 WjsDhtDriver;
  #define WJS_HT_NAME "DHT11"
#endif

class WjsHTSensor {
public:
  WjsHTSensor(uint8_t sensorPin = 52, uint32_t sampleIntervalMs = 2000UL)
    : _pin{sensorPin}, _intervalMs{sampleIntervalMs < 2000UL ? uint32_t(2000UL) : sampleIntervalMs}
  {}

  bool begin()
  {
    _startMs = millis();
    _lastReadMs = _startMs - _intervalMs;           // read on first update
    _lastGoodMs = _startMs;
    _state = SensorState::STATE_READY;
    _valid = false; _errorCount = 0;
    return true;
  }

  SensorState updateAndGetState()
  {
    if (_state == SensorState::STATE_OFF) return _state;
    const uint32_t now = millis();
    const uint32_t interval = (_state == SensorState::STATE_ERROR) ? ERROR_RETRY_MS : _intervalMs;
    if (now - _lastReadMs >= interval)
    {
      _lastReadMs = now;
      float t = 0.0f, h = 0.0f;
      byte data[40];
      const int err = _sensor.read2(_pin, &t, &h, data);
      if (err == SimpleDHTErrSuccess && plausible(t, h))
      {
        _errorCount = 0; _valid = true; _lastGoodMs = now;
        _lastTemp = t; _lastHumid = h;
        _state = SensorState::STATE_READY;
      }
      else
      {
        _lastErr = err;
        if (_errorCount < 255) ++_errorCount;
        if (_errorCount >= 3) _state = SensorState::STATE_ERROR;
      }
    }
    if (_valid && now - _lastGoodMs > STALE_MS) _valid = false;
    return _state;
  }

  /** true + values only when a recent plausible reading exists. */
  bool getReading(float& tempC, float& rhPct) const
  {
    if (!_valid) return false;
    tempC = _lastTemp; rhPct = _lastHumid;
    return true;
  }

  float getTemperature() const { return _valid ? _lastTemp  : NAN; }
  float getHumidity()    const { return _valid ? _lastHumid : NAN; }
  SensorState getState() const { return _state; }
  unsigned long getTimeElapsed() const { return (_state == SensorState::STATE_OFF) ? 0UL : (millis() - _startMs); }
  void setSampleInterval(uint32_t ms) { if (ms >= 2000UL) _intervalMs = ms; }

  void printDashboard(Print& out) const
  {
    out.print(F("[")); out.print(Wjs::stateName(_state)); out.print(F(" " WJS_HT_NAME " | ")); Wjs::printHM(out, getTimeElapsed());
    out.print(F("] Temp=")); Wjs::printNum(out, getTemperature(), 1);
    out.print(F("C RH="));   Wjs::printNum(out, getHumidity(), 1); out.print('%');
    if (_state == SensorState::STATE_ERROR) { out.print(F(" FAULT: read err ")); out.print(_lastErr); }
    out.println();
  }

private:
  static constexpr uint32_t STALE_MS       {30000UL};
  static constexpr uint32_t ERROR_RETRY_MS {5000UL};

  uint8_t  _pin;
  uint32_t _intervalMs;
  WjsDhtDriver _sensor;
  float _lastTemp{0.0f}, _lastHumid{0.0f};
  SensorState _state{SensorState::STATE_OFF};
  uint8_t  _errorCount{0};
  int      _lastErr{0};
  bool     _valid{false};
  uint32_t _startMs{0}, _lastReadMs{0}, _lastGoodMs{0};

  /** Physical range of the part (DHT11: 0..50 C / 20..90 %; DHT22: -40..80 C / 0..100 %); reject garbage. */
  static bool plausible(float t, float h)
  {
#ifdef WJS_HT_DHT22
    return t > -40.0f && t < 80.0f && h >= 0.0f && h <= 100.0f;
#else
    return t > -5.0f && t < 55.0f && h >= 5.0f && h <= 100.0f;
#endif
  }
};

#endif // WJS_HT_SENSOR_H
