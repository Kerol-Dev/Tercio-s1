#pragma once
#include <Arduino.h>

class SensorsADC {
public:
  struct ThermistorCfg {
    float R0      = 10000.0f;
    float T0_C    = 25.0f;
    float Beta    = 3435.0f;
    float Rfixed  = 10000.0f;
    bool  pullup  = true;
  };
  struct VbusCfg {
    float Rtop = 91000.0f;
    float Rbot = 10000.0f;
  };

  bool begin(float vref = 3.3f, uint8_t adcBits = 12);

  void setThermistor(const ThermistorCfg& c);
  void setVbusDivider(const VbusCfg& c);
  void setLpfAlpha(float a);
  void setVref(float vref);

  void update();

  // Readouts
  float vbus_V() const;
  float temperatureC() const;

  // Raw helpers
  uint32_t rawPB0() const;
  uint32_t rawPB1() const;
  float nodeVoltagePB1() const;

private:
  // config
  float _vref{3.3f};
  uint32_t _adcMax{4095};
  float _alpha{0.2f};
  ThermistorCfg _th{};
  VbusCfg _vb{};

  // state
  float _vbus_V{0.0f};
  float _temp_C{25.0f};
  uint32_t _rawPB0{0};
  uint32_t _rawPB1{0};
};