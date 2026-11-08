#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

/*
  AppConfig.h - everything you may want to change lives here: pins, addresses, defaults, EEPROM map.

  WIRING SUMMARY (Arduino Mega 2560)
    A0 / A1 / A2     MEMS analog outputs: H2 / CH4 / CO
    Pin 52           DHT11/DHT22 data
    SDA 20 / SCL 21  I2C bus: H2S sensor (0x74) + five GP8600 4-20 mA DACs (0x58..0x5C), 10k pull-ups to 5 V
    RX1 19 / TX1 18  SEN0220 infrared CO2 sensor (Serial1)
    RX2 17 / TX2 16  Nextion display (Serial2)
    USB (Serial)     text dashboard or Serial Plotter
*/

namespace Cfg {

  // ------------------------------------------------------------------ USB serial
  constexpr unsigned long USB_BAUD = 115200;
  constexpr bool     PLOT_MODE     = false;       // true: Arduino Serial Plotter lines, false: text dashboards
  constexpr uint32_t REPORT_MS     = 1000;




  // ------------------------------------------------------------------ bench testing
  // >0: shorten ALL warm-ups to this many seconds so the screen/outputs can be tested on the bench.
  // Readings are NOT valid then, and MEMS R0 would be calibrated in room air. Set back to 0 afterwards!
  constexpr uint32_t BENCH_WARMUP_S = 0;





  // ------------------------------------------------------------------ pins
  constexpr uint8_t PIN_H2   = A0;
  constexpr uint8_t PIN_CH4  = A1;
  constexpr uint8_t PIN_CO   = A2;
  constexpr uint8_t PIN_DHT  = 52;                // digital pin
  constexpr int8_t  PIN_BUZZER      = -1;         // free digital pin = buzzer that sounds while any alarm is active (-1 = none)
  constexpr int8_t  PIN_ALARM_RELAY = -1;         // free digital pin = relay/lamp output, ON while any alarm is active





  // ------------------------------------------------------------------ electrical
  constexpr float VREF            = 5.01f;         // ADC reference of the Mega
  constexpr float SENSOR_SUPPLY_V = 5.01f;         // voltage on the MEMS boards' supply: MEASURE IT with a multimeter!
                                                  // (it can also be edited later on the "Supply Vc" row of each MEMS page)




  // ------------------------------------------------------------------ warm-up policy
  constexpr uint16_t H2S_INACTIVE_HOURS  = 24;    // hours the H2S MODULE was unpowered before this start (0 once it runs continuously)
  constexpr int      MEMS_STORAGE_MONTHS = 0;     // months the MEMS sensors were stored unpowered (0 / 1-6 / >6)
  constexpr bool     H2S_LIB_TEMP_COMP   = false; // DFRobot library H2S compensation is wrong above 20 C - keep false (see WjsH2S.h)





  // ------------------------------------------------------------------ I2C addresses
  constexpr uint8_t ADDR_H2S    = 0x74;           // SEL switch = I2C, A1=0 A0=0
  constexpr uint8_t ADDR_DAC_H2 = 0x58, ADDR_DAC_H2S = 0x59, ADDR_DAC_CO = 0x5A, ADDR_DAC_CO2 = 0x5B, ADDR_DAC_CH4 = 0x5C; // 4 to 20 mA outputs

  // ------------------------------------------------------------------ channels
  // The order is the order of the Nextion pages: page 1 = CO, 2 = CO2, 3 = H2S, 4 = H2, 5 = CH4
  constexpr uint8_t N_GAS = 5;
  enum GasIndex : uint8_t { G_CO = 0, G_CO2 = 1, G_H2S = 2, G_H2 = 3, G_CH4 = 4 };




  // First-start defaults. Everything here can be changed on the screen afterwards and is then remembered.
  // (ppm that gives 20 mA on the 4-20 mA output; 4 mA = 0 ppm)
  constexpr float DACHI_CO  = 1000.0f,  DACHI_CO2 = 50000.0f, DACHI_H2S = 100.0f, DACHI_H2 = 1000.0f, DACHI_CH4 = 10000.0f;
  // Alarm levels - PLACEHOLDERS: set the limits required for YOUR site (0 = alarm off).
  constexpr float ALARM_CO  = 30.0f,    ALARM_CO2 = 5000.0f,  ALARM_H2S = 5.0f,   ALARM_H2 = 100.0f,  ALARM_CH4 = 1000.0f;

  // ------------------------------------------------------------------ EEPROM map (Mega: 4 KB). One 80-byte slot per channel.
  constexpr int EE_CHANNEL_BASE = 0;              // channel i uses EE_CHANNEL_BASE + 80 * i   (0 .. 399)
  constexpr int EE_MQ7          = 400;            // WjsMQ7.h  (optional, not used by default)
  constexpr int EE_CO2_ANALOG   = 420;            // WjsCO2.h  (optional, not used by default)

  // ------------------------------------------------------------------ Nextion
  constexpr unsigned long NEX_BAUD       = 115200; // must match "baud=115200" in the HMI's Program.s
  constexpr uint32_t NEX_HOME_REFRESH_MS = 500;    // Home page refresh
  constexpr uint32_t NEX_SET_REFRESH_MS  = 1000;   // settings page refresh (labels + live line)
  constexpr uint8_t  NEX_WRITES_PER_PASS = 3;      // component writes per loop() pass (keeps loop() responsive)
  constexpr uint32_t CONFIRM_WINDOW_MS   = 5000UL; // second press (ZERO / DEFAULTS) must follow within this time
  constexpr uint32_t MESSAGE_MS          = 6000UL; // how long a message stays on the status line

  // Nextion pages (must match the page order in the HMI): 0 = Home, 1..5 = the five sensors above
  enum Page : uint8_t { PAGE_HOME = 0, PAGE_FIRST_SENSOR = 1 };

  // Nextion touch-event trigger IDs. In the HMI the button code is:  printh 23 02 54 <hex id>
  enum Trigger : uint8_t {
    T_APPLY    = 1,        // 0x01  APPLY & SAVE : reads ALL rows of the current sensor page in one go
    T_RELOAD   = 2,        // 0x02  RELOAD       : puts the stored values back on the screen
    T_ZERO     = 3,        // 0x03  ZERO         : re-zero the MEMS sensor in clean air (press twice)
    T_DEFAULTS = 4         // 0x04  DEFAULTS     : restore factory constants of this sensor (press twice)
  };

  constexpr const char* FW_VERSION = "GasMonitor Version 3";
}

#endif
