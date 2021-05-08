#pragma once

#include "pico/stdlib.h"
// required if multiple devices on SPI bus
// CS must be set to 0 to communicate
// see SSD1351::beginTransaction()
#define HAS_CS_PIN 0

#include <functional>

class Transaction
{
public:
  Transaction(std::function<void()> endFunc = {})
    : _endFunc(endFunc)
  {
  }

  ~Transaction()
  {
    reset();
  }
  
  void reset()
  {
    if (_endFunc)
      _endFunc();
  }

private:
  std::function<void()> _endFunc;
};


class SSD1351
{
public:
#if HAS_CS_PIN
  SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst, uint gpio_cs);
  Transaction beginTransaction();
#else
  SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst);
#endif
  ~SSD1351();

  void init();

  void waitTransfer();
  void setStartLine(uint8_t l);
  
  // \param wait if false, doesn't wait for finish, CPU can work on other things while transferring. It is advisable to call \see waitTransfer() before the next communication with the display starts
  // color fmt:[15:11]=b; [10:5]=g; [0:4]=r
  void fillRect(uint x0, uint y0, uint x1, uint y1, uint16_t color, bool wait=true);

  enum StandByType
  {
    Off,
    Weak, // just display off
    Strong, // display and Vregulator off
  };

  void standBy(StandByType t);
  
  enum ScrollSpeed
  {
    NormalSpeed  =1,
    ReducedSpeed =2,
    SlowestSpeed =3
  };
  void horizontalScroll(uint8_t numSteps, uint8_t startRow, uint8_t numRows, ScrollSpeed spd);
  
private:

  char _priv[36];
};