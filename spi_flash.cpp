/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Example of reading/writing an external serial flash using the PL022 SPI interface

#include "SSD1351.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"


#define SPI_MOSI_DISPLAY  19
#define SPI_SCK_DISPLAY   18
#define SPI_DC_DISPLAY    17
#define SPI_RST_DISPLAY   16

uint8_t v[3][2] = { 
  { 31 << 3, 0 }, // red
  { 7, 7<<5 },  // green
  { 0, 31 } // blue
};

uint8_t x0 = 0, y0 = 0, x1 = 127, y1 = 127;

SSD1351 disp(SPI_MOSI_DISPLAY, SPI_SCK_DISPLAY, SPI_DC_DISPLAY, SPI_RST_DISPLAY);
int main() {
  // Enable UART so we can print status output
  stdio_init_all();

  uint8_t c = 0;
  disp.fillRect(0, 0, 127, 127, 0);
//  printf("initDma");
//  disp.initDma();
//  printf(" done\n");

  auto absTime = get_absolute_time();
  while (x0 < x1)
  {
    //sleep_ms(2000);
    //printf("fill..");
    disp.fillRect(x0++, y0++, x1--, y1--, *(uint16_t*)v[c]);
    c = (c + 1) % 3;
    //printf("filled\n");
    //sleep_ms(500);
  }
  auto tDist = absolute_time_diff_us(absTime, get_absolute_time());
  printf("Filling display took %d us\n", (uint)tDist);
      return 0;
}
