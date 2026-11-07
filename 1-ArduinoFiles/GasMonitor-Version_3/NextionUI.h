#ifndef NEXTION_UI_H
#define NEXTION_UI_H

#include <Arduino.h>
#include <EasyNextionLibrary.h>
#include "AppConfig.h"
#include "WjsDTypes.h"
#include "GasChannel.h"
#include "HTSensor.h"

/*
  NextionUI.h - everything the Nextion screen does, in plain words
  ================================================================
  The screen has SIX pages:
      pgHome  (page 0)   one line per gas: concentration, unit, 4-20 mA output, state; plus temperature/humidity.
                         Each gas line has a button that opens that gas's settings page.
      pgCO, pgCO2, pgH2S, pgH2, pgCH4  (pages 1..5)
                         the SETTINGS page of one sensor: up to 16 rows (name | value | unit), a live status line
                         and the buttons APPLY & SAVE, RELOAD, ZERO, DEFAULTS, HOME.
  The five settings pages are IDENTICAL copies. The Arduino knows which sensor a page belongs to from the page number,
  so it fills in the row names, values and units itself.

  HOW THE TWO SIDES TALK (EasyNextionLibrary)
    Arduino -> screen : writeStr("pgCO.tV3.txt", "12.5")      sets the text of a box
                        writeNum("pgCO.tState.bco", 2016)     sets a colour
    screen  -> Arduino: when a page opens, its Preinitialize code sends  printh 23 02 50 <page number>
                        (the library stores it in myNex.currentPageId)
                        when a button is released it sends  printh 23 02 54 <id>  and the library calls trigger<id>()
                        (the four trigger functions live at the bottom of the .ino file).
    Reading back      : readStr("pgCO.tV3.txt") asks the screen for the text of a box. It is used ONLY inside APPLY
                        (the screen is certainly connected then, because you just touched it).

  WHAT HAPPENS WHEN YOU PRESS "APPLY & SAVE"  (trigger 1, ONE trigger per sensor page)
    1. read the text of every row of this page from the screen            (validate: must be numbers)
    2. for every row whose text you changed: channel.setParam(row, value) (clamped to the allowed range)
    3. save all constants of this sensor to EEPROM (one block, with checksum)
    4. write the stored (possibly clamped) values back to the screen and show a message
  Rows you did not touch are NOT written back, so a value the sensor changed on its own (slow R0 drift) is never
  overwritten with an old number.

  TIMING: the screen is updated a few components per loop() pass (NEX_WRITES_PER_PASS) so loop() never stalls, and only
  the page that is showing is refreshed. If the screen is unplugged nothing blocks - the writes just go nowhere.
*/

// ---------------------------------------------------------------------------- colours (RGB565 numbers, as Nextion wants)
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) { return uint16_t(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)); }
constexpr uint16_t COL_OK    = rgb565(0, 150, 0);      // green
constexpr uint16_t COL_WARM  = rgb565(230, 190, 0);    // yellow
constexpr uint16_t COL_ALARM = rgb565(220, 0, 0);      // red
constexpr uint16_t COL_FAULT = rgb565(235, 120, 0);    // orange
constexpr uint16_t COL_CAL   = rgb565(0, 90, 220);     // blue
constexpr uint16_t COL_OFF   = rgb565(110, 110, 110);  // grey

class NextionUI {
public:
  static constexpr uint8_t SLOTS = GasChannel::MAX_PARAMS;     // rows on a settings page

  NextionUI(EasyNex& nex, GasChannel* channels, WjsHTSensor* env)
    : _nex{nex}, _ch{channels}, _env{env} {}

  void begin(unsigned long baud) { _nex.begin(baud); }

  /** Call every loop(): listens for touches (may call a trigger function) and refreshes the visible page. */
  void service(uint32_t now)
  {
    _nex.NextionListen();

    int pg = _nex.currentPageId;
    if (pg < 0 || pg > Cfg::N_GAS) pg = Cfg::PAGE_HOME;
    if (pg != _page)                                   // a different page opened
    {
      _page = uint8_t(pg);
      _cycleIdx = 0; _fillIdx = 0; _armed = 0; _msgUntilMs = 0;
    }

    if (_msgUntilMs && now > _msgUntilMs) _msgUntilMs = 0;
    if (_armed && now > _armedUntilMs) _armed = 0;

    if (_page == Cfg::PAGE_HOME) serviceHome(now); else serviceSettings(now);
  }

  // ================================================================ the four button actions
  /** trigger 1: APPLY & SAVE for the sensor on the page that is showing. */
  void onApply()
  {
    GasChannel* c = currentChannel(); if (!c) return;
    const uint32_t now = millis();
    const uint8_t n = c->paramCount();
    float typed[SLOTS];
    char path[32];

    for (uint8_t i = 0; i < n; ++i)                    // step 1: read + validate ALL rows first
    {
      snprintf(path, sizeof(path), "%s.tV%u.txt", pageName(), i);
      const String s = _nex.readStr(path);
      ParamInfo p; c->paramInfo(i, p);
      if (s == "ERROR") { say("Screen did not answer - try again", now); return; }
      if (!Wjs::parseFloat(s.c_str(), typed[i]))
      {
        char lab[14]; Wjs::copyFlash(lab, sizeof(lab), p.label);
        char m[40]; snprintf(m, sizeof(m), "%s: not a number", lab); say(m, now);
        return;                                        // nothing was changed
      }
    }

    uint8_t changed = 0;                               // step 2: apply only the rows you edited
    for (uint8_t i = 0; i < n; ++i)
      if (typed[i] != _filled[i]) { c->setParam(i, typed[i]); ++changed; }

    if (changed) c->save();                            // step 3: one EEPROM block per sensor
    char m[48];
    if (!c->configOk())  snprintf(m, sizeof(m), "Saved, but 20mA value must be > 4mA value!");
    else if (changed)    snprintf(m, sizeof(m), "Saved (%u changed)", changed);
    else                 snprintf(m, sizeof(m), "No changes");
    say(m, now);
    _fillIdx = 0;                                      // step 4: show the stored values
  }

  /** trigger 2: put the stored values back on the screen (throws away unsaved typing). */
  void onReload() { if (!currentChannel()) return; _fillIdx = 0; say("Values reloaded", millis()); }

  /** trigger 3: ZERO. Press twice within 5 s. Re-zeroes a MEMS sensor in CLEAN AIR. */
  void onZero()
  {
    GasChannel* c = currentChannel(); if (!c) return;
    const uint32_t now = millis();
    if (!confirmed(1, now)) { say("Clean air? Press ZERO again", now); return; }
    if (c->sensor().recalibrate()) say("Zeroing 10 s - keep clean air", now);
    else                           say("ZERO not possible now / not supported", now);
  }

  /** trigger 4: DEFAULTS. Press twice within 5 s. Restores factory constants (R0 is kept). */
  void onDefaults()
  {
    GasChannel* c = currentChannel(); if (!c) return;
    const uint32_t now = millis();
    if (!confirmed(2, now)) { say("Restore defaults? Press again", now); return; }
    c->resetDefaults(); c->save(); _fillIdx = 0;
    say("Defaults restored (R0 kept)", now);
  }

private:
  EasyNex&     _nex;
  GasChannel*  _ch;
  WjsHTSensor* _env;

  uint8_t  _page{0};
  uint8_t  _cycleIdx{0};
  uint8_t  _fillIdx{0};                 // 0..SLOTS-1 = still filling value boxes; SLOTS = done
  uint32_t _cycleDoneMs{0};
  float    _filled[SLOTS];              // the numbers last written to the value boxes (to detect what you edited)
  uint8_t  _armed{0};                   // 1 = ZERO pressed once, 2 = DEFAULTS pressed once
  uint32_t _armedUntilMs{0};
  char     _msg[48]{0};
  uint32_t _msgUntilMs{0};
  bool     _msgPending{false};

  // ------------------------------------------------------------------ helpers
  const char* pageName() const
  {
    static const char* const n[Cfg::N_GAS + 1] = {"pgHome", "pgCO", "pgCO2", "pgH2S", "pgH2", "pgCH4"};
    return n[_page];
  }

  GasChannel* currentChannel() { return (_page >= Cfg::PAGE_FIRST_SENSOR && _page <= Cfg::N_GAS) ? &_ch[_page - Cfg::PAGE_FIRST_SENSOR] : nullptr; }

  void say(const char* s, uint32_t now)
  {
    strncpy(_msg, s, sizeof(_msg) - 1); _msg[sizeof(_msg) - 1] = '\0';
    _msgUntilMs = now + Cfg::MESSAGE_MS;
    _msgPending = true;                                // shown on the very next pass, not at the next refresh
  }

  bool confirmed(uint8_t what, uint32_t now)
  {
    if (_armed == what && now <= _armedUntilMs) { _armed = 0; return true; }
    _armed = what; _armedUntilMs = now + Cfg::CONFIRM_WINDOW_MS;
    return false;
  }

  void setTxt(const char* comp, const char* text)
  {
    char p[34]; snprintf(p, sizeof(p), "%s.%s.txt", pageName(), comp); _nex.writeStr(p, text);
  }
  void setCol(const char* comp, uint16_t color)
  {
    char p[34]; snprintf(p, sizeof(p), "%s.%s.bco", pageName(), comp); _nex.writeNum(p, color);
  }

  static void chip(const GasChannel& c, char* txt, uint8_t size, uint16_t& color)
  {
    switch (c.sensor().getState())
    {
      case SensorState::STATE_READY:       strncpy(txt, "OK",    size); color = COL_OK;    break;
      case SensorState::STATE_ALARM:       strncpy(txt, "ALARM", size); color = COL_ALARM; break;
      case SensorState::STATE_ERROR:       strncpy(txt, "FAULT", size); color = COL_FAULT; break;
      case SensorState::STATE_CALIBRATING: strncpy(txt, "CALIB", size); color = COL_CAL;   break;
      case SensorState::STATE_WARMING_UP:  strncpy(txt, "WARM",  size); color = COL_WARM;  break;
      default:                             strncpy(txt, "N/A",   size); color = COL_OFF;   break;
    }
    txt[size - 1] = '\0';
  }

  /** "12.34 mA" or "-- mA" when the DAC module does not answer. */
  static void mAText(const GasChannel& c, char* buf, uint8_t size)
  {
    if (!c.dac().isLinkOk()) { strncpy(buf, "-- mA", size); buf[size - 1] = '\0'; return; }
    char n[12]; Wjs::fmtFloat(n, sizeof(n), float(c.dac().getLastTargetMA()), 2);
    snprintf(buf, size, "%s mA", n);
  }

  /** Summary for the Home page header: ALARM <gases> / FAULT <gases> / WARMING UP / ALL OK. */
  void banner(char* txt, uint8_t size, uint16_t& color) const
  {
    char alarm[24] = "", fault[24] = ""; bool warm = false;
    for (uint8_t g = 0; g < Cfg::N_GAS; ++g)
    {
      const SensorState s = _ch[g].sensor().getState();
      char* list = (s == SensorState::STATE_ALARM) ? alarm : (s == SensorState::STATE_ERROR ? fault : nullptr);
      if (list) { const uint8_t l = strlen(list); if (l + 6 < 24) snprintf(list + l, 24 - l, " %s", _ch[g].code()); }
      if (s == SensorState::STATE_WARMING_UP || s == SensorState::STATE_CALIBRATING) warm = true;
    }
    if (alarm[0])      { snprintf(txt, size, "ALARM%s", alarm); color = COL_ALARM; }
    else if (fault[0]) { snprintf(txt, size, "FAULT%s", fault); color = COL_FAULT; }
    else if (warm)     { strncpy(txt, "WARMING UP", size);      color = COL_WARM;  }
    else               { strncpy(txt, "ALL OK", size);          color = COL_OK;    }
    txt[size - 1] = '\0';
  }

  // ================================================================== HOME page
  static constexpr uint8_t HOME_FIELDS = Cfg::N_GAS * 5 + 4;

  void serviceHome(uint32_t now)
  {
    if (_cycleIdx < HOME_FIELDS)
    {
      for (uint8_t n = 0; n < Cfg::NEX_WRITES_PER_PASS && _cycleIdx < HOME_FIELDS; ++n) writeHomeField(_cycleIdx++);
      if (_cycleIdx >= HOME_FIELDS) _cycleDoneMs = now;
    }
    else if (now - _cycleDoneMs >= Cfg::NEX_HOME_REFRESH_MS) _cycleIdx = 0;
  }

  void writeHomeField(uint8_t i)
  {
    char comp[12], buf[28]; uint16_t col;
    if (i < Cfg::N_GAS * 5)
    {
      const GasChannel& c = _ch[i / 5];
      switch (i % 5)
      {
        case 0: snprintf(comp, sizeof(comp), "t%sv", c.code()); c.valueText(buf, sizeof(buf)); setTxt(comp, buf); break;
        case 1: snprintf(comp, sizeof(comp), "t%su", c.code()); Wjs::copyFlash(buf, sizeof(buf), c.unitLabel()); setTxt(comp, buf); break;
        case 2: snprintf(comp, sizeof(comp), "t%sm", c.code()); mAText(c, buf, sizeof(buf)); setTxt(comp, buf); break;
        case 3: snprintf(comp, sizeof(comp), "t%ss", c.code()); chip(c, buf, sizeof(buf), col); setTxt(comp, buf); break;
        default:snprintf(comp, sizeof(comp), "t%ss", c.code()); chip(c, buf, sizeof(buf), col); setCol(comp, col); break;
      }
      return;
    }
    float t, h;
    switch (i - Cfg::N_GAS * 5)
    {
      case 0: if (_env && _env->getReading(t, h)) { char n[10]; Wjs::fmtFloat(n, sizeof(n), t, 1); snprintf(buf, sizeof(buf), "%s C", n); } else strncpy(buf, "-- C", sizeof(buf));
              setTxt("tTemp", buf); break;
      case 1: if (_env && _env->getReading(t, h)) snprintf(buf, sizeof(buf), "%d %%RH", int(h + 0.5f)); else strncpy(buf, "-- %RH", sizeof(buf));
              setTxt("tHum", buf); break;
      case 2: banner(buf, sizeof(buf), col); setTxt("tBanner", buf); break;
      default:banner(buf, sizeof(buf), col); setCol("tBanner", col); break;
    }
  }

  // ================================================================== SETTINGS pages
  // per refresh cycle: header (name, state, state colour, live line, status line) + 16 row names + 16 row units
  static constexpr uint8_t SET_FIELDS = 5 + SLOTS + SLOTS;

  void serviceSettings(uint32_t now)
  {
    GasChannel* c = currentChannel(); if (!c) return;

    if (_msgPending) { _msgPending = false; setTxt("tMsg", _msg); return; }   // button feedback appears immediately

    if (_fillIdx < SLOTS)                              // priority: put the stored values into the value boxes
    {
      for (uint8_t n = 0; n < Cfg::NEX_WRITES_PER_PASS && _fillIdx < SLOTS; ++n) fillValue(*c, _fillIdx++);
      return;
    }
    if (_cycleIdx < SET_FIELDS)
    {
      for (uint8_t n = 0; n < Cfg::NEX_WRITES_PER_PASS && _cycleIdx < SET_FIELDS; ++n) writeSettingsField(*c, _cycleIdx++);
      if (_cycleIdx >= SET_FIELDS) _cycleDoneMs = now;
    }
    else if (now - _cycleDoneMs >= Cfg::NEX_SET_REFRESH_MS) _cycleIdx = 0;
  }

  /** Writes the value box of row i and remembers the number, so APPLY can tell which rows you edited. */
  void fillValue(GasChannel& c, uint8_t i)
  {
    char comp[8], buf[16];
    snprintf(comp, sizeof(comp), "tV%u", i);
    ParamInfo p;
    if (i < c.paramCount() && c.paramInfo(i, p))
    {
      Wjs::fmtFloat(buf, sizeof(buf), c.getParam(i), p.decimals);
      if (!Wjs::parseFloat(buf, _filled[i])) _filled[i] = NAN;          // remember what the TEXT says (rounded)
    }
    else { buf[0] = '\0'; _filled[i] = NAN; }
    setTxt(comp, buf);
  }

  void writeSettingsField(GasChannel& c, uint8_t i)
  {
    char comp[8], buf[48]; uint16_t col;
    if (i < 5)
    {
      switch (i)
      {
        case 0: setTxt("tName", c.title()); break;
        case 1: chip(c, buf, sizeof(buf), col); setTxt("tState", buf); break;
        case 2: chip(c, buf, sizeof(buf), col); setCol("tState", col); break;
        case 3: { char v[16], u[8], m[16]; c.valueText(v, sizeof(v)); Wjs::copyFlash(u, sizeof(u), c.unitLabel()); mAText(c, m, sizeof(m));
                  snprintf(buf, sizeof(buf), "%s %s | %s", v, u, m); setTxt("tLive", buf); break; }
        default:
                if (_msgUntilMs) setTxt("tMsg", _msg);
                else { c.sensor().getStatusLine(buf, sizeof(buf)); setTxt("tMsg", buf); }
                break;
      }
      return;
    }
    i -= 5;
    const bool isUnit = i >= SLOTS; const uint8_t row = isUnit ? uint8_t(i - SLOTS) : i;
    snprintf(comp, sizeof(comp), isUnit ? "tU%u" : "tL%u", row);
    ParamInfo p;
    if (row < c.paramCount() && c.paramInfo(row, p)) Wjs::copyFlash(buf, sizeof(buf), isUnit ? p.unit : p.label);
    else buf[0] = '\0';
    setTxt(comp, buf);
  }
};

#endif // NEXTION_UI_H
