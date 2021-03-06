/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Example of reading/writing an external serial flash using the PL022 SPI interface

#include "SSD1351.h"
#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#define SPI_MOSI_DISPLAY  19
#define SPI_SCK_DISPLAY   18
#define SPI_DC_DISPLAY    17
#define SPI_RST_DISPLAY   16

extern void initHeartBeat();


constexpr uint8_t HSVlights[] = 
{  0,   4,   8,  13,  17,  21,  25,  30,  34,  38,  42,  47,  51,  55,  59,  64,
  68,  72,  76,  81,  85,  89,  93,  98, 102, 106, 110, 115, 119, 123, 127, 132,
 136, 140, 144, 149, 153, 157, 161, 166, 170, 174, 178, 183, 187, 191, 195,
 200, 204,208, 212, 217, 221, 225, 229, 234, 238, 242, 246, 251, 255};

constexpr uint Hue2RGB(int angle)
{
  uint8_t red=0, green=0, blue=0;
  angle %= 360;
  if (angle<60)  {red = 255; green = HSVlights[angle];     blue = 0;} else
  if (angle<120) {red = HSVlights[120-angle]; green = 255; blue = 0;} else 
  if (angle<180) {red =   0, green = 255; blue = HSVlights[angle-120];} else 
  if (angle<240) {red =   0, green = HSVlights[240-angle]; blue = 255;} else 
  if (angle<300) {red = HSVlights[angle-240], green = 0;   blue = 255;} else 
                 {red = 255, green = 0; blue = HSVlights[360-angle];} 
  return (uint)red | (green << 8) | (blue << 16) ;
}

uint RGB24ToRGB565(uint rgb24)
{
  return (((rgb24 >> 16) * 0x1F / 0xFF) << 11)
       | ((((rgb24 >> 8) & 0xFF) * 0x3F / 0xFF) << 5)
       | ((rgb24 & 0xFF) * 0x1F / 0xFF);
}

uint8_t v[3][2] = { 
  { 31 << 3, 0 }, // red
  { 7, 7<<5 },  // green
  { 0, 31 } // blue
};

#define RIGHTPOS 127

uint8_t x0 = 0, y0 = 0, x1 = RIGHTPOS, y1 = 127;
SSD1351 disp(SPI_MOSI_DISPLAY, SPI_SCK_DISPLAY, SPI_DC_DISPLAY, SPI_RST_DISPLAY);


// decreasing sys clk to 82 MHz allows to use divider 2 (instead of 4) for SPI and therefore increasing SPI bauds from 31.5MHz to 41MHz (maximum for SSD1351 display)
void change_peri_clk()
{
  #define PLL_SYS_KHZ (74 * 1000)
  set_sys_clock_khz(PLL_SYS_KHZ, true);
  clock_configure(clk_peri, 
    0,
    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
    clock_get_hz(clk_sys),
    clock_get_hz(clk_sys));
}

void core1()
{
  while (true)
  {
    tight_loop_contents();
  }
}

int main() {
  sleep_ms(20); // for SPI display to suppress bouncing effects when pressing reset
  // Enable UART so we can print status output
//  change_peri_clk();
  stdio_init_all();
//  multicore_launch_core1(core1);
  vreg_set_voltage(VREG_VOLTAGE_0_90);
//    setup_default_uart();

  initHeartBeat();
  disp.init();


  printf("initialized\n");
  printf("Measured SPI freq: %u kHz\n", frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI));
  printf("set baudrate = %d\n", spi_get_baudrate(spi0));

  disp.setFillColor(0x12 << 11);
  for (uint i = 0; true; i = (i + 1) % 4)
  {
    printf("Rotation = %d\n", i);
    disp.setRotation(static_cast<SSD1351::Rotation>(static_cast<int>(SSD1351::Rotation::DEGREE_0) + i));

    disp.setDrawColor(0);
    disp.drawLine(10, 40, 60, 40, 0);
    disp.drawLine(60, 40, 60, 30, 0);
    disp.drawLine(60, 30, 110, 55, 0);
    disp.drawLine(110, 55, 60, 80, 0);
    disp.drawLine(60, 80, 60, 70, 0);
    disp.drawLine(60, 70, 10, 70, 0);
    disp.drawLine(10, 70, 10, 40, 0);
    sleep_ms(100);
//    i = 3;
  }

  while (true) ;
  for (uint i = 0; true; ++i)
  {
    i = i % 4;
    y0 = 0;
    disp.setRotation((SSD1351::Rotation)i, true);
    disp.setFillColor(0x12 << 11);
    disp.fillRect(0, y0, RIGHTPOS/2, y0 + 6); y0 += 13;
    disp.setFillColor(0x14 << 0);
    disp.fillRect(0, y0, RIGHTPOS, y0 + 6); y0 += 13;
    sleep_ms(80);
  }

  while (true) ;
  bool a = false;

  a = y1;
  printf("the value of a is %d\n", a);
  disp.setWindow(10, 10, 49, 49);
  {
    auto pixelPath = disp.beginPixelPath();
    for (int i = 0; i < 40 * 40; ++i) {
      disp.setDrawColor((i & 1) * 4555);
      disp.setPixels();
    }
  }

  {
    disp.setDrawColor(0x1F << 0);
    auto pixelPath = disp.beginPixelPath();
    disp.setPixels(40);
  }
  //  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x08<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x0A<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x10<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x14<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x18<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x1C<<11); y0 += 13;
//  disp.fillRect(0, y0, RIGHTPOS, y0 + 6, 0x1F<<11); y0 += 13;
  while (true);
  uint8_t c = 0;
  //  disp.fillRect(0, 0, RIGHTPOS, SSD1351_WIDTH, 0);
//  printf("initDma");
//  disp.initDma();
//  printf(" done\n");

  //auto absTime = get_absolute_time();
  int i = 0;
//  while (i<100)
    {
  //    //sleep_ms(2000);
  //    //printf("fill..");
     //disp.fillRect(x0++, y0++, x1--, y1--, *(uint16_t*)v[c]);
 //     disp.fillRect(x0, y0, x1, y1, *(uint16_t*)v[c]);
  //    c = (c + 1) % 3;
  //    //printf("filled\n");
  //    sleep_ms(10000);
   //   i++;
     }
    //auto tDist = absolute_time_diff_us(absTime, get_absolute_time());
    //printf("Filling display took %d us\n", (uint)tDist/100);
  uint16_t l = 0;
  uint clrStart = 200;
  while(true)
  {
//      absTime = get_absolute_time();
      for (uint16_t y0 = 0; y0 < 128; y0=y0+1) {
        uint rgb24 = Hue2RGB(359*y0 / (SSD1351::WIDTH - 1) + clrStart);
        disp.waitTransfer();
        disp.setFillColor(RGB24ToRGB565(rgb24));
        disp.fillRect(0, y0, 127, y0);
      }
//    auto tDist = absolute_time_diff_us(absTime, get_absolute_time());
//    printf("Filling display took %d us\n", (uint)tDist);
//    stdio_flush();
//    sleep_ms(1);
//    //clr++;
    clrStart = (clrStart + 360 / SSD1351::WIDTH) % 360;
  }
  uint16_t clr = 0;
  while (true)
  {
    disp.setStartLine(l);
    disp.setFillColor( clr++);
    disp.fillRect(0, l, SSD1351::WIDTH - 1, l);
    l = (l + 1) % SSD1351::WIDTH;
    sleep_ms(10);
  }
  return 0;
  }
