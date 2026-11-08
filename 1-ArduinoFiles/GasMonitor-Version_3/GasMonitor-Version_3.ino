/*
  ======================================================================================================
   GasMonitor - Version 3          Arduino Mega 2560 gas analyser with Nextion touch screen
  ======================================================================================================

  WHAT IT DOES (in plain words)
  -----------------------------
  Five gas sensors are measured all the time:
        CO   (MEMS sensor, analog pin)          CO2 (infrared sensor SEN0220, serial port)
        H2S  (DFRobot electrochemical, I2C)     H2  (MEMS sensor, analog pin)
        CH4  (MEMS sensor, analog pin)
  A DHT11/DHT22 measures temperature and humidity (used to correct the MEMS sensors, and shown on the screen).
  Each gas has its own 4-20 mA output (a GP8600 module on the I2C bus): 4 mA = 0 ppm, 20 mA = the "20mA value" you set.
  If a sensor is warming up or has a fault, its output goes to 3.6 mA (below 4 mA) so a PLC can tell
  "fault" from "0 ppm". The Nextion screen shows everything and lets you change the constants of each sensor.

  HOW THE PROGRAM IS BUILT - 4 simple ideas
  -----------------------------------------
  1. ONE OBJECT PER THING.  Every sensor is a C++ "class" in its own file (WjsMEMS.h, WjsH2S.h, WjsCO2DFIR.h ...).
     Every class has the same functions, so the rest of the program treats all sensors the same way:
        updateAndGetState()  - do a little work; call it every loop()
        getReading(ppm)      - is there a valid concentration? (true/false) and what is it
        paramCount/getParam/setParam - the adjustable constants of this sensor (what the settings page edits)
  2. A "CHANNEL" JOINS THE PARTS.  GasChannel (GasChannel.h) = sensor + its 4-20 mA output + alarm level + unit +
     4mA/20mA values + output calibration, and it saves all of those to EEPROM in one block.
  3. NOTHING EVER WAITS.  loop() has no delay(). Each class is a small "state machine" (WARMING_UP -> CALIBRATING ->
     READY / ALARM / ERROR) that does a tiny step every time loop() runs and remembers where it is.
     So a missing or broken module can never freeze the program: it just goes to ERROR and is retried every 2 s.
  4. THE SCREEN IS A SEPARATE LAYER.  NextionUI.h draws Home and the settings pages from the channel objects.
     It never talks to a sensor directly - only to GasChannel.

  WHAT HAPPENS IN setup()  (runs once after power-on / reset)
  -----------------------------------------------------------
     a) start USB serial and the I2C bus (with a timeout, so a dead I2C device cannot hang the Arduino)
     b) load the saved constants of every channel from EEPROM (R0, curves, alarm, 4/20 mA values, unit, DAC c0/k0).
        Nothing saved yet (first start) -> the built-in defaults from AppConfig.h are used.
     c) start every sensor (begin()). Sensors that are not connected are NOT a problem - they report ERROR.
     d) start every 4-20 mA output module and put it at the fault level until a valid reading exists
     e) start the Nextion library

  WHAT HAPPENS IN loop()  (runs thousands of times per second)
  ------------------------------------------------------------
     1. updateHardware()   every sensor, DAC module and the DHT does its small step.
     2. ui.service()       the screen: listen for touches, refresh the page that is showing (a few boxes per pass).
     3. every 250 ms:      read all concentrations, drive the 4-20 mA outputs, switch buzzer/relay,
                           and save to EEPROM when a sensor changed a constant by itself (finished a zero calibration).
     4. handleSerial()     commands typed on the USB serial monitor (type h for help).
     5. report()           print the dashboard (or Serial Plotter line) once per second. The dashboard ends with one
                           [NEX] line: which page the screen shows, button counters, APPLY results, boxes written.

  SCREEN STATUS ON THE USB SERIAL MONITOR (115200 baud)
  -----------------------------------------------------
     [NEX] lines show what the screen side is doing: page changes, which button was pressed, every row that APPLY
     read from the screen and changed (old -> new), and every message shown on the screen.
     Type  n  for a full status plus a live check that the display answers. Type  v  to switch the event log on/off.
     (The log is automatically off in Serial Plotter mode.)

  WHAT HAPPENS WHEN YOU TOUCH THE SCREEN
  --------------------------------------
     * Buttons that only change page (HOME, CO, CO2 ...) are handled by the display alone - the Arduino is told the
       new page number and starts drawing it.
     * APPLY & SAVE / RELOAD / ZERO / DEFAULTS each send one "trigger" number. EasyNextionLibrary calls the
       matching triggerN() function at the bottom of this file, which calls NextionUI (see NextionUI.h).

  Libraries to install (Arduino IDE: Sketch > Include Library > Manage Libraries):
     DFRobot_MultiGasSensor, DFRobot_GP8XXX (and its dependency DFRobot_RTU), SimpleDHT, EasyNextionLibrary
*/

#include <Wire.h>
#include <EEPROM.h>
#include <EasyNextionLibrary.h>
#include "AppConfig.h"
#include "WjsMEMS.h"
#include "WjsH2S.h"
#include "WjsCO2DFIR.h"
#include "WjsGP8600.h"
#include "HTSensor.h"
#include "GasChannel.h"
#include "NextionUI.h"

using namespace Cfg;

// ============================================================================ THE SENSORS
//                  analog pin  profile (curve data)   storage   Vref  steps   supply voltage    vol%?  decimals(ppm)
WjsMEMS CO_mems  { PIN_CO,  WjsMemsData::CO,  MEMS_STORAGE_MONTHS, VREF, 1024.0f, SENSOR_SUPPLY_V, false, 1 };
WjsMEMS H2_mems  { PIN_H2,  WjsMemsData::H2,  MEMS_STORAGE_MONTHS, VREF, 1024.0f, SENSOR_SUPPLY_V, true,  1 };
WjsMEMS CH4_mems { PIN_CH4, WjsMemsData::CH4, MEMS_STORAGE_MONTHS, VREF, 1024.0f, SENSOR_SUPPLY_V, true,  0 };
WjsH2S      H2S_DFR { ADDR_H2S, H2S_INACTIVE_HOURS, true };   // I2C address, hours the module was unpowered, do warm-up
WjsIRCO2    CO2_IR  { Serial1, true };                        // serial port, do the 3-minute preheat
WjsHTSensor DHT     { PIN_DHT };                              // temperature + humidity only (no 4-20 mA output)

// ============================================================================ THE 4-20 mA OUTPUT MODULES
// c0 and k0 = calibration numbers from the Excel sheet "DAC-output-calibration" (steps = c0 * nominal + k0).
// They are only the FIRST-START values: after you edit them on the screen the saved ones are used.
//                     I2C address    c0             k0
WjsGP8600 CO_DAC  { ADDR_DAC_CO,  0.997123669, 113.134877950 };   // DAC #3
WjsGP8600 CO2_DAC { ADDR_DAC_CO2, 0.99884748,  98.763288417  };   // DAC #4
WjsGP8600 H2S_DAC { ADDR_DAC_H2S, 1.000529118, 76.547367081  };   // DAC #2
WjsGP8600 H2_DAC  { ADDR_DAC_H2,  1.000769815, 96.790931004  };   // DAC #1
WjsGP8600 CH4_DAC { ADDR_DAC_CH4, 0.99999999,  103.198193202 };   // DAC #5










// ============================================================================ THE CHANNELS (order = Nextion pages 1..5)
//                  code   long name           sensor    output    EEPROM address               alarm       20 mA at
GasChannel channels[N_GAS] = {
  { "CO",  "Carbon monoxide", CO_mems,  CO_DAC,  EE_CHANNEL_BASE + 0 * GasChannel::SLOT_BYTES, ALARM_CO,  DACHI_CO  },
  { "CO2", "Carbon dioxide",  CO2_IR,   CO2_DAC, EE_CHANNEL_BASE + 1 * GasChannel::SLOT_BYTES, ALARM_CO2, DACHI_CO2 },
  { "H2S", "Hydrogen sulfide",H2S_DFR,  H2S_DAC, EE_CHANNEL_BASE + 2 * GasChannel::SLOT_BYTES, ALARM_H2S, DACHI_H2S },
  { "H2",  "Hydrogen",        H2_mems,  H2_DAC,  EE_CHANNEL_BASE + 3 * GasChannel::SLOT_BYTES, ALARM_H2,  DACHI_H2  },
  { "CH4", "Methane",         CH4_mems, CH4_DAC, EE_CHANNEL_BASE + 4 * GasChannel::SLOT_BYTES, ALARM_CH4, DACHI_CH4 },
};
WjsMEMS* const memsSensors[3] = { &CO_mems, &H2_mems, &CH4_mems };   // the three that use temperature/humidity compensation











// ============================================================================ THE NEXTION SCREEN
EasyNex myNex(Serial2);                       // the display is on Serial2 (pins 16/17)
NextionUI ui(myNex, channels, &DHT);

// The four buttons that need the Arduino. In the Nextion Editor each button's "Touch Release Event" contains
//     printh 23 02 54 01      (APPLY & SAVE)    printh 23 02 54 02   (RELOAD)
//     printh 23 02 54 03      (ZERO)            printh 23 02 54 04   (DEFAULTS)
// The library turns that into a call of trigger1() ... trigger4():
void trigger1() { ui.onApply();    }          // APPLY & SAVE: read every row of the page, apply, save, show result
void trigger2() { ui.onReload();   }          // RELOAD: show the saved values again
void trigger3() { ui.onZero();     }          // ZERO: re-zero the MEMS sensor in clean air (press twice)
void trigger4() { ui.onDefaults(); }          // DEFAULTS: restore this sensor's factory constants (press twice)













// ============================================================================ SMALL PROGRAM STATE
bool     plotMode      = PLOT_MODE;           // can be switched from the serial monitor (p / d)
double   forcedMA      = -1.0;                // >= 0: all 4-20 mA outputs forced to this current (bench test, command "t 12")
uint32_t forcedUntilMs = 0;
uint32_t lastLogicMs   = 0, lastReportMs = 0;
char     cmdBuf[16];  uint8_t cmdLen = 0;     // collects a command typed on the USB serial monitor












// ============================================================================ STEP 1 OF LOOP: let every device do its small step
void updateHardware()
{
  for (uint8_t g = 0; g < N_GAS; ++g) channels[g].sensor().updateAndGetState();     // sensors
  for (uint8_t g = 0; g < N_GAS; ++g) channels[g].dac().updateAndGetState();        // 4-20 mA modules
  DHT.updateAndGetState();

  float t, h;
  if (DHT.getReading(t, h))                      // only a VALID temperature/humidity is passed on to the compensation
    for (uint8_t i = 0; i < 3; ++i) memsSensors[i]->setEnvironment(t, h);
}














// ============================================================================ STEP 3 OF LOOP (every 250 ms)
void runChannels(uint32_t now)
{
  bool anyAlarm = false;

  if (forcedMA >= 0.0 && now > forcedUntilMs) forcedMA = -1.0;                      // bench test ends after 5 minutes

  for (uint8_t g = 0; g < N_GAS; ++g)
  {
    GasChannel& c = channels[g];
    c.update();                                                                      // read the concentration
    c.driveOutput(forcedMA);                                                         // set the 4-20 mA output
    if (c.sensor().getState() == SensorState::STATE_ALARM) anyAlarm = true;

    if (c.sensor().consumeParamsChanged()) c.save();                                 // e.g. a ZERO calibration just finished
  }

  if (PIN_ALARM_RELAY >= 0) digitalWrite(PIN_ALARM_RELAY, anyAlarm ? HIGH : LOW);
  if (PIN_BUZZER >= 0)      digitalWrite(PIN_BUZZER, (anyAlarm && ((now / 400UL) & 1UL)) ? HIGH : LOW);   // beeps while alarm
}












// ============================================================================ STEP 4 OF LOOP: commands from the USB serial monitor
void printHelp()
{
  Serial.println(F("Commands (end with Enter):"));
  Serial.println(F("  p        Serial Plotter mode        d   text dashboard mode"));
  Serial.println(F("  n        Nextion screen status + live check that the display answers (waits ~1 s if no display)"));
  Serial.println(F("  v        screen event log on/off (page changes, button presses, what APPLY read and changed)"));
  Serial.println(F("  c        zero-calibrate all MEMS sensors (CLEAN AIR only!)"));
  Serial.println(F("  x        erase ALL saved settings (defaults + full preheat after the next restart)"));
  Serial.println(F("  t 12     force all 4-20 mA outputs to 12 mA for 5 minutes (bench test)   t off  = stop"));
  Serial.println(F("  h        this help"));
}

void runCommand(const char* cmd)
{
  if (cmd[0] == 'p')      { plotMode = true;  ui.setLogEnabled(false); }   // the Serial Plotter cannot read text lines
  else if (cmd[0] == 'd') { plotMode = false; ui.setLogEnabled(true);  }
  else if (cmd[0] == 'n') ui.printStatus(Serial, true);                      // status + probe: asks the display which page it shows
  else if (cmd[0] == 'v') { ui.setLogEnabled(!ui.logEnabled()); Serial.println(ui.logEnabled() ? F("Screen event log ON") : F("Screen event log OFF")); }
  else if (cmd[0] == 'h') printHelp();
  else if (cmd[0] == 'c') { for (uint8_t i = 0; i < 3; ++i) memsSensors[i]->recalibrate(); Serial.println(F("MEMS zero-calibration started (clean air!)")); }
  else if (cmd[0] == 'x') { for (uint8_t g = 0; g < N_GAS; ++g) channels[g].eraseStored(); Serial.println(F("Saved settings erased - restart the Arduino")); }
  else if (cmd[0] == 't')
  {
    float mA;
    if (Wjs::parseFloat(cmd + 1, mA) && mA >= 0.0f && mA <= 20.0f) { forcedMA = mA; forcedUntilMs = millis() + 300000UL; Serial.print(F("Outputs forced to ")); Serial.print(mA, 2); Serial.println(F(" mA for 5 min")); }
    else { forcedMA = -1.0; Serial.println(F("Output test off")); }
  }
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    const char c = char(Serial.read());
    if (c == '\n' || c == '\r') { if (cmdLen) { cmdBuf[cmdLen] = '\0'; runCommand(cmdBuf); cmdLen = 0; } }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
}

// ============================================================================ STEP 5 OF LOOP: report once per second
float plotVal(uint8_t g) { return channels[g].valid() ? channels[g].ppm() : 0.0f; }

void report(uint32_t now)
{
  if (now - lastReportMs < REPORT_MS) return;
  lastReportMs = now;

  if (plotMode)
  {
    // Arduino Serial Plotter wants ONE line per sample: "label:value" pairs separated by commas, numbers only.
    // Values are in ppm. A gas that is not ready plots as 0. CO2 is divided by 100 so it shares a readable scale.
    Serial.print(F("CO:"));         Serial.print(plotVal(G_CO), 1);
    Serial.print(F(",CO2_div100:"));Serial.print(plotVal(G_CO2) / 100.0f, 1);
    Serial.print(F(",H2S:"));       Serial.print(plotVal(G_H2S), 1);
    Serial.print(F(",H2:"));        Serial.print(plotVal(G_H2), 1);
    Serial.print(F(",CH4:"));       Serial.println(plotVal(G_CH4), 0);
    return;
  }

  Serial.println(F("-----------------------------------------------------"));
  for (uint8_t g = 0; g < N_GAS; ++g) channels[g].sensor().printDashboard(Serial);
  DHT.printDashboard(Serial);
  for (uint8_t g = 0; g < N_GAS; ++g) channels[g].dac().printDashboard(Serial);
  ui.printStatus(Serial);                        // one line about the Nextion screen (never blocks)
}

















// ============================================================================ setup()
void setup()
{
  Serial.begin(USB_BAUD);

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);               // a stuck/unplugged I2C device can never freeze loop(); the bus is reset on timeout
#endif

  if (PIN_BUZZER >= 0)      { pinMode(PIN_BUZZER, OUTPUT);      digitalWrite(PIN_BUZZER, LOW); }
  if (PIN_ALARM_RELAY >= 0) { pinMode(PIN_ALARM_RELAY, OUTPUT); digitalWrite(PIN_ALARM_RELAY, LOW); }

  // b) saved constants first (so a stored R0 gives a quick warm-up and the stored unit/alarm are active from the start)
  for (uint8_t g = 0; g < N_GAS; ++g) channels[g].load();

  // c) every begin() is non-blocking and tolerates a missing module
  H2S_DFR.setLibraryTempCompensation(H2S_LIB_TEMP_COMP);
  for (uint8_t i = 0; i < 3; ++i) memsSensors[i]->begin();
  H2S_DFR.begin();
  CO2_IR.begin(9600);
  DHT.begin();

  // d) outputs start at the fault level (3.6 mA) until a valid reading exists
  for (uint8_t g = 0; g < N_GAS; ++g) { channels[g].dac().begin(); channels[g].dac().setFaultOutput(); }

  if (BENCH_WARMUP_S > 0)                        // bench testing only (see AppConfig.h)
  {
    for (uint8_t i = 0; i < 3; ++i) memsSensors[i]->setWarmupMs(BENCH_WARMUP_S * 1000UL);
    H2S_DFR.setWarmupMs(BENCH_WARMUP_S * 1000UL);
    CO2_IR.setWarmupMs(BENCH_WARMUP_S * 1000UL);
  }

  ui.begin(NEX_BAUD);                            // e) Nextion
  ui.setLog(&Serial);                            //    screen events are also printed on the USB serial port ...
  ui.setLogEnabled(!plotMode);                   //    ... except in Serial Plotter mode  if (!plotMode) printHelp();
}

















// ============================================================================ loop()
void loop()
{
  const uint32_t now = millis();

  updateHardware();                              // 1. sensors, outputs, DHT
  ui.service(now);                               // 2. screen

  if (now - lastLogicMs >= 250UL)                // 3. concentrations, 4-20 mA, alarms (4 times per second)
  {
    lastLogicMs = now;
    runChannels(now);
  }
  handleSerial();                                // 4. USB commands
  report(now);                                   // 5. dashboard / plotter
}
