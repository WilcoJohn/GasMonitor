#include "WjsGP8600.h"


double TheoreticalOutVal_mA[]{ 2, 4, 8, 10, 12, 16, 18, 20 };
int output_length{ 8 };


//              Address     c0             k0
WjsGP8600 wjs_DAC{0x58
};
// to calibration values set the c0 = 1, k0 = 0


void setup() {
  Serial.begin(115200);
  wjs_DAC.begin();
}


void loop() {
  Serial.println();
  for (int idx{ 0 }; idx < output_length; ++idx) {
    Serial.print("Setting to: '");
    Serial.print(TheoreticalOutVal_mA[idx]);
    Serial.println(" mA");
    wjs_DAC.setmAOutput(TheoreticalOutVal_mA[idx]);
    delay(14000);
  }
} 





















