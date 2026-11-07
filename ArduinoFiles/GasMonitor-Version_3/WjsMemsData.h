#ifndef WJS_MEMS_DATA_H
#define WJS_MEMS_DATA_H

#include <stdint.h>

/*
  Sensor profiles for Winsen MEMS MOX gas sensors on DFRobot Fermion boards (RL = 4.7 kOhm).
  Numbers are unchanged from the original WjsDTypes.h: curves/compensation are read off datasheet plots
  (approximate). Re-fit them in the Excel sheet if you need better accuracy.

  NOTE (DFRobot SEN0564 wiki): these MEMS boards are QUALITATIVE sensors and DFRobot says they must not be
  used in personal-safety systems. The CO board is specified 5-5000 ppm but the fitted curve below only
  covers up to 1000 ppm (maxPPM clamps there).
*/

struct MemsCompTable {
  const float* tempC; uint8_t nT;
  const float* rhPct; uint8_t nRH;
  const float* k;                 // row-major [T][RH]
  float refT, refRH;              // conditions where Rs0 was measured
};

struct MemsProfile {
  const char* name;
  float rLoad;                    // Board load resistor, kOhm
  float aLow,  bLow;              // Curve for Rs/R0 >= splitRatio (low ppm)
  float aHigh, bHigh;             // Curve for Rs/R0 <  splitRatio (high ppm)
  float splitRatio;               // 0 = single curve
  float noGasRatio;               // Rs/R0 at/above this -> 0 ppm
  float maxPPM;                   // Clamp
  uint16_t warmupH[3];            // Preheat hours: <1 month, 1-6 months, >6 months
  const MemsCompTable* comp;
};

namespace WjsMemsData {

  // CH4, H2S, CO share one Fig4 (ref 20C/55%RH)
  static const float T_STD[]  = {-10, 0, 10, 20, 30, 40, 50};
  static const float RH_STD[] = {30, 60, 85};
  static const float K_STD[]  = {1.71f, 1.48f, 1.28f,  1.57f, 1.32f, 1.15f,  1.46f, 1.28f, 1.09f,
                                 1.38f, 1.09f, 0.91f,  1.13f, 0.98f, 0.86f,  1.01f, 0.87f, 0.71f,
                                 0.88f, 0.72f, 0.67f};
  static const MemsCompTable COMP_STD = {T_STD, 7, RH_STD, 3, K_STD, 20.0f, 55.0f};

  // H2 has its own Fig4 (ref 20C/65%RH)
  static const float T_H2[]   = {10, 20, 30, 40, 50};
  static const float RH_H2[]  = {40, 65, 85};
  static const float K_H2[]   = {1.22f, 1.07f, 0.85f,  1.13f, 0.99f, 0.75f,  1.06f, 0.88f, 0.68f,
                                 0.99f, 0.79f, 0.61f,  0.85f, 0.73f, 0.58f};
  static const MemsCompTable COMP_H2 = {T_H2, 5, RH_H2, 3, K_H2, 20.0f, 65.0f};

  //                               name   RL     aLow      bLow       aHigh     bHigh      split  noGas maxPPM  warmup h        comp
  static const MemsProfile CH4 = {"CH4", 4.7f, 82.19108f, -6.01283f, 82.19108f, -6.01283f, 0.00f, 0.97f, 10000, {48, 72, 168}, &COMP_STD};
  static const MemsProfile H2S = {"H2S", 4.7f, 0.2418f,   -3.4136f,  2.2398f,   -1.4147f,  0.32f, 1.00f, 50,    {48, 72, 168}, &COMP_STD};
  static const MemsProfile CO  = {"CO",  4.7f, 0.5481f,   -2.9927f,  0.5020f,   -2.9206f,  0.28f, 1.00f, 1000,  {24, 48, 72},  &COMP_STD};
  static const MemsProfile H2  = {"H2",  4.7f, 0.6106f,   -3.7995f,  9.0142f,   -1.5504f,  0.30f, 1.00f, 1000,  {24, 48, 72},  &COMP_H2};

} // namespace WjsMemsData

#endif
