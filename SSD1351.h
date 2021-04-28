#pragma once

#include "pico/stdlib.h"

class SSD1351
{
public:
  SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst, uint gpio_cs=0);
  ~SSD1351();
  
  void fillRect(uint x0, uint y0, uint x1, uint y1, uint16_t color);
  
  enum StandByType
  {
    Off,
    Weak, // just display off
    Strong, // display and Vregulator off
  };
  
  void standBy(StandByType t);
  
#if REMEMBER_DMA
  bool initDma();
  void uninitDma();
#endif  
private:
  
  char _priv[16];
};