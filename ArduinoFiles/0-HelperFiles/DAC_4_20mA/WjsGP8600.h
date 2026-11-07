#include <Wire.h>
#include <DFRobot_GP8XXX.h>

class WjsGP8600 {
  public:
    // Safe settling delay constant for loop execution timing
    static const uint8_t SETTLING_DELAY_MS = 20; 

    // Constructor: Uses initializer list to map dependencies and parameters
    WjsGP8600(uint8_t address, double conv_slope = 1, double conv_intercept = 0)
      : m_DAC{&Wire, address},
        m_slope{conv_slope},
        m_intercept{conv_intercept},
        p_address{address}
    {}

    // Initializes the single bus and establishes DAC settings
    void begin() {
      Wire.begin(); 
      Serial.println(F("Starting GP8600 initialization..."));
      
      while (m_DAC.begin() != 0) {
        Serial.print(F("Hardware not detected: '0x"));
        Serial.print(p_address, HEX);
        Serial.println(F("'"));
        delay(1000); 
      }
      
      // Enforce the physical current mode hardware configuration limit
      m_DAC.setDACOutRange(m_DAC.eOutputRange20MA);
      Serial.print(F("'0x"));
      Serial.print(p_address, HEX);
      Serial.print(F("'set to "));
      Serial.print(static_cast<int>(MIN_CURRENT_MA));
      Serial.print(F(".."));
      Serial.print(static_cast<int>(MAX_CURRENT_MA));
      Serial.println(F(" mA!"));
    }

    // Function 1: Native engineering double unit math variant (e.g. 0 to 65535 space conversion)
    void setOutputValue(double engineeringInput) {
      double rawDacCalculated = (m_slope * engineeringInput) + m_intercept;
      
      // Clamp values strictly within the structural hardware 16-bit register limits
      if (rawDacCalculated < 0.0)     rawDacCalculated = 0.0;
      if (rawDacCalculated > 65535.0) rawDacCalculated = 65535.0;

      m_DAC.setDACOutData(static_cast<uint16_t>(rawDacCalculated));
    }

    // Function 2: Directly drive the module based on pure milliamp inputs (4.0 to 20.0 mA)
    void setmAOutput(double target_mA) {
      // Safe boundary clamps for industrial loop standards
      if (target_mA < MIN_CURRENT_MA) target_mA = MIN_CURRENT_MA;
      if (target_mA > MAX_CURRENT_MA) target_mA = MAX_CURRENT_MA;

      // 0 to 20mA maps linearly across the full 16-bit space (65535 steps)
      double calculatedSteps = (target_mA / 20.0) * 65535.0;
      
     setOutputValue(calculatedSteps);
    }

  private:
    DFRobot_GP8600_I2C m_DAC;
    double m_slope;
    double m_intercept;
    
    // Safety boundaries for pure mA calculations
    static constexpr double MIN_CURRENT_MA = 4.0;  
    static constexpr double MAX_CURRENT_MA = 20.0;
    const uint8_t p_address{};
};