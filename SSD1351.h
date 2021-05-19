#pragma once

#include "Transaction.hpp"
#include "pico/stdlib.h"

// required if multiple devices on SPI bus
// CS must be set to 0 to communicate
// see SSD1351::beginTransaction()
#define NEED_CS_PIN 0

class SSD1351
{
public:
#if NEED_CS_PIN
  SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst, uint gpio_cs);
  Transaction beginTransaction();
#else
  SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst);
#endif
  ~SSD1351();

  static const uint8_t WIDTH = 128;
  static const uint8_t HEIGHT = 128;

  void init();

  void waitTransfer();

  void setStartLine(uint l);
  void invertDisplay(bool inverted);

  void setWindow(uint8_t x0 = 0, uint8_t y0 = 0, uint8_t x1 = WIDTH, uint8_t y1 = HEIGHT);

  enum class Rotation : uint8_t
  {
    DEGREE_0,
    DEGREE_90,
    DEGREE_180,
    DEGREE_270,
  };
  void setRotation(Rotation rot, bool clearScreen = true);
  enum class Mirror : uint8_t
  {
    NO,
    HORIZONTAL,
    VERTICAL,
    BOTH,
  };
  void setMirror(Mirror m, bool clearScreen = true);

  void setAngle(uint8_t angle); // min. angle = 360/256 = 1.41°

  void setDrawColor(uint16_t clr);
  void setFillColor(uint16_t clr);

  // start drawing pixels within the given window and the given increment mode within the applied window
  Transaction beginPixelPath();

  void setPixels(uint16_t numPixels = 1);  // pixels with drawColor written at current internal address

  void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);

  //! helper function which calls \see setWindow() + \see fill()
  // \param wait if false, doesn't wait for finish, CPU can work on other things while transferring. It is advisable to call \see waitTransfer() before the next communication with the display starts
  // color fmt:[15:11]=b; [10:5]=g; [0:4]=r
  void fillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool wait = true);

  // fills the current window with fillColor
  void fill(bool wait = true);

  void bitblt(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t const* data, bool wait = true);

  enum class StandBy : uint8_t
  {
    OFF,
    ON,   // just display off
    DEEP, // display and Vregulator off
  };

  void standBy(StandBy t);

  enum class ScrollSpeed : uint8_t
  {
    NormalSpeed  = 1,
    ReducedSpeed = 2,
    SlowestSpeed = 3
  };
  void horizontalScroll(uint8_t numSteps, uint8_t startRow, uint8_t numRows, ScrollSpeed spd);

private:

  char _priv[44];
};