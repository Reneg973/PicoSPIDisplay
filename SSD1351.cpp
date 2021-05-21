#include "SSD1351.h"
#include "DMA.hpp"
#include "gpio.hpp"
#include "SPI.hpp"
#include "util.hpp"

#include "hardware/interp.h"
#include "hardware/spi.h"
#include <new>
#include <stdio.h>

// shared commands
#define SSD_CMD_SETCOLUMN           0x15 // 2Byte: Set start and end column of active video RAM area
#define SSD_CMD_SETROW              0x75 // 2Byte: Set start and end row of active video RAM area
#define SSD_CMD_HORIZSCROLL         0x96 // 5 byte command; A=# of scroll; B=start row addr; C=num rows to scroll; E=scroll interval: 1=normal; 3=slowest
#define SSD_CMD_SETREMAP            0xA0 // Remap various display settings, like hardware mapping and most importantly color mode
                                          // A[0]: Horizontal or vertical increment; A[1]: map column address to 0 or 127; A[2]: Swap color sequence (A->B->C : C->B->A)
                                          //A[4]: COM Scan direction; A[5]: COM Split Odd/Even; A[7:6] color depth=65k; 262k fmt1; 262k fmt2 
#define REMAP_VERTICAL_INC            0b01
#define REMAP_COL_ADDRESS             0b10
#define REMAP_SWAP_COLOR_SEQUENCE    0b100
#define REMAP_SCAN_DIRECTION       0b10000
#define REMAP_SPLIT_ODD_EVEN      0b100000

#define SSD_CMD_STARTLINE           0xA1 // Set display start line, needs to be set to 96 for 128x96 displays
#define SSD_CMD_DISPLAYOFFSET       0xA2 // Set display offset (hardware dependent, needs to be set to 0; locked)
#define SSD_CMD_DISPLAYALLON        0xA5
#define SSD_CMD_INVERSEDISPLAY      0xA7
#define SSD_CMD_NOOP                0xAD
#define SSD_CMD_DISPLAY_OFF         0xAE
#define SSD_CMD_DISPLAY_ON          0xAF
#define SSD_CMD_CLOCKDIV            0xB3 // A[7:4]: Fosc freq; A[3:0] Div
#define SSD_CMD_SETGRAY             0xB8 //when setting this, a list of grayscale table entries must follow (GS1..GS63)
#define SSD_CMD_USEBUILTIN_LUT      0xB9 //reset gray lut set by SETGRAY to linear
#define SSD_CMD_PRECHARGELEVEL      0xBB // 0 = 0.44 x Vcc; ... 3E = 0.83 x Vcc
#define SSD_CMD_VCOMH               0xBE // COM deselect voltage level (see precharge)
#define SSD_CMD_COMMANDLOCK         0xFD // SSD1351 has more bits defined; see datasheet

// SSD1331D Regs
#define SSD1331_CMD_DRAW_LINE       0x21 // 7 bytes: A: col0; B: row0; C: col1; D: row1; E: color C; F: color B; G: color A
#define SSD1331_CMD_FILL_RECT       0x22 // 10 bytes: A: col0; B: row0; C: col1; D: row1; E..G: linecolor; H..J: fillcolor
#define SSD1331_CMD_COPY_RECT       0x23 // 6 bytes: A: col0; B: row0; C: col1; D: row1; E: colNew; F: rowNew
#define SSD1331_CMD_DIM_RECT        0x24 // 4 bytes: A: col0; B: row0; C: col1; D: row1; reduces intensity of given area
#define SSD1331_CMD_CLEAR_RECT      0x25 // 4 bytes: A: col0; B: row0; C: col1; D: row1
#define SSD1331_CMD_ENABLEFILL_RECT 0x26 // 0: default unfilled; 1: use fill color for fill rect cmd
#define SSD1331_CMD_CONTINUOUS_SCROLL_SETUP 0x27 // 5 bytes; A=horiz. scroll offset columns; B: start row; C: numRowsToScroll; D: vert. scroll offset; E: interval 0..3
#define SSD1331_STOP_SCROLL         0x2E
#define SSD1331_START_SCROLL        0x2F
#define SSD1331_SET_CONTRAST_A      0x81
#define SSD1331_SET_CONTRAST_B      0x82
#define SSD1331_SET_CONTRAST_C      0x83
#define SSD1331_CMD_CONTRASTMASTER  0x87 // see SSD1351_CMD_CONTRASTMASTER
#define SSD1331_CMD_PRECHARGE2
#define SSD1331_CMD_NORMALDISPLAY   0xA4
#define SSD1331_CMD_DISPLAYALLOFF   0xA6
#define SSD1331_CMD_MUXRATIO        0xA8
#define SSD1331_CMD_DIM_MODE_SETTING  0xAB // 5 bytes: A: reserved (0); B: contrast A; C: contrast B; D: contrast C; E: precharge range 0..31
#define SSD1331_CMD_DISPLAY_DIM     0xAC
#define SSD1331_CMD_POWERSAVE_ON    0xB0  // default = 1A (enabled); disabled = 0B
#define SSD1331_CMD_PHASE_PERIOD_ADJUSTMENT       0xB1

// SSD1351D Regs
#define SSD1351_CMD_WRITERAM        0x5C
#define SSD1351_CMD_READRAM         0x5D
#define SSD1351_CMD_STOPSCROLL      0x9E
#define SSD1351_CMD_STARTSCROLL     0x9F
#define SSD1351_CMD_DISPLAYALLOFF   0xA4
#define SSD1351_CMD_NORMALDISPLAY   0xA6
#define SSD1351_CMD_FUNCTIONSELECT  0xAB // A[0]: Disable internal Vdd (power save)
#define SSD1351_CMD_PRECHARGE       0xB1  // locked by default
#define SSD1351_CMD_DISPLAYENHANCE  0xB2 // 3Byte command; 0:0:0 = normal; A4:0:0 = enhance display performance
#define SSD1351_CMD_SETVSL          0xB4 // 3Byte: A0 = internal VSL; B+C constant
#define SSD1351_CMD_SETGPIO         0xB5 // no usage on SPI display
#define SSD1351_CMD_PRECHARGE2      0xB6 // 0 = invalid: 1=1DCLK; 2=2DCLK; F=15DCLK
#define SSD1351_CMD_CONTRASTABC     0xC1 // 3 bytes
#define SSD1351_CMD_CONTRASTMASTER  0xC7 // reduce current output; 0:1/16 to F:16/16
#define SSD1351_CMD_MUXRATIO        0xCA

#define INITIAL_REMAP_VALUE   0b01100000//REMAP_SCAN_DIRECTION | REMAP_SPLIT_ODD_EVEN

namespace
{

#define SPI_BAUDRATE 38 * 1000 * 1000

struct SSD1351Priv
{
  SSD1351Priv(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst
#if NEED_CS_PIN
    , uint gpio_cs
#endif
    )
    : _dc(gpio_dc)
    , _spiOut(gpio_mosi, gpio_sck, SPI_BAUDRATE, 8, SPI_CPOL_1, SPI_CPHA_1)
    , _rst(gpio_rst)
#if NEED_CS_PIN
    , _cs(gpio_cs)
#endif
    , _swapCoords(0b00010000)
    , _clrDraw(0)
    , _clrFill(0)
  {
  }
  
  void sendCommand(uint8_t cmd)
  {
    _dc << false;
    _spiOut.write(cmd);
  }

  void sendCommand(uint8_t cmd, uint8_t d1)
  {
    sendCommand(cmd);
    _dc << true;
    _spiOut.write(d1);
  }

  void sendCommand(uint8_t cmd, std::initializer_list<uint8_t> const& data)
  {
    sendCommand(cmd);
    _dc << true;
    _spiOut.write(data);
  }

  //Enable writing to the SSD1351 RAM
  void enableWrite()
  {
    sendCommand(SSD1351_CMD_WRITERAM);
    _dc << true;
  }

  void setRegion(uint8_t x0 = 0, uint8_t y0 = 0, uint8_t x1 = SSD1351::WIDTH - 1, uint8_t y1 = SSD1351::HEIGHT - 1)
  {
    inplace_minmax(x0, x1);
    inplace_minmax(y0, y1);
    if (_swapCoords & REMAP_VERTICAL_INC)
    {
      swap(x0, y0);
      swap(x1, y1);
    }

    if (x0 != _x0 || x1 != _x1 || y0 != _y0 || y1 != _y1)
    {
      sendCommand(SSD_CMD_SETROW, {y0, y1});
      sendCommand(SSD_CMD_SETCOLUMN, {x0, x1});
      _regionSize = ((uint16_t)x1 - x0 + 1) * ((uint16_t)y1 - y0 + 1);
      _x0 = x0; _x1 = x1; _y0 = y0; _y1 = y1;
    }
  }

  void initDisplay()
  {
    _spiOut.set_baudrate(SPI_BAUDRATE);
#define STABLE_RESET_TIME_US  40 // even if datasheet says 2us, this is not stable when resetting
    _rst << true;
    sleep_us(STABLE_RESET_TIME_US);
    _rst << false;
    sleep_us(STABLE_RESET_TIME_US);
    _rst << true;
    sleep_us(STABLE_RESET_TIME_US);
#if 1
#define COMMAND_LOCK_INIT1 0x12 // "Unlock OLED driver IC MCU interface from entering command"
#define COMMAND_LOCK_INIT2 0xB1 // "Command A2,B1,B3,BB,BE, C1 accessible if in unlock state"  
#else
#define COMMAND_LOCK_INIT1 0x16
#define COMMAND_LOCK_INIT2 0x12
#endif
  //  writeCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT1);     // this is the initial value after reset
    sendCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT2);     // set command lock
#undef COMMAND_LOCK_INIT1
#undef COMMAND_LOCK_INIT2
//    sendCommand(SSD_CMD_DISPLAY_OFF);          // 0xAE

    sendCommand(SSD_CMD_CLOCKDIV, 0x2F);      // 0xB3 // higher=increased power consumption; lower=may flicker / 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    sendCommand(SSD1351_CMD_MUXRATIO, SSD1351::HEIGHT - 1);      // number rows to be muxed
    sendCommand(SSD_CMD_SETREMAP, INITIAL_REMAP_VALUE);  //RGB format; do not split odd+even
    sendCommand(SSD_CMD_STARTLINE, 0);
    sendCommand(SSD_CMD_DISPLAYOFFSET, 0x0);
//    writeCommand(SSD1351_CMD_PRECHARGE, 0x32);
//    writeCommand(SSD_CMD_VCOMH, 0x05);
    sendCommand(SSD1351_CMD_NORMALDISPLAY);
//    sendCommand(SSD1351_CMD_CONTRASTABC, 0xC8, 0xC8, 0xC8);
    sendCommand(SSD1351_CMD_CONTRASTMASTER, 2);
    sendCommand(SSD1351_CMD_DISPLAYENHANCE, { 0xA4, 0, 0 });
    sendCommand(SSD1351_CMD_SETVSL, {0xA0, 0xB5, 0x55});
    sendCommand(SSD1351_CMD_PRECHARGE2, 0x0F);

    setRegion();
    fill();    // clear VRam before activating display

    sendCommand(SSD_CMD_DISPLAY_ON);
  }

  void initDisplay2()
  {
    _spiOut.set_baudrate(SPI_BAUDRATE);
#define STABLE_RESET_TIME_US  40 // even if datasheet says 2us, this is not stable when resetting
    _rst << true;
    sleep_us(STABLE_RESET_TIME_US);
    _rst << false;
    sleep_us(STABLE_RESET_TIME_US);
    _rst << true;
    sleep_us(STABLE_RESET_TIME_US);

#define COMMAND_LOCK_INIT1 0x16
#define COMMAND_LOCK_INIT2 0x12
    sendCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT1);     // set command lock
    sendCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT2);     // set command lock
#undef COMMAND_LOCK_INIT1
#undef COMMAND_LOCK_INIT2
//    sendCommand(SSD_CMD_DISPLAY_OFF);           // 0xAE

  //  sendCommand(SSD_CMD_CLOCKDIV, 0x2F);       // 0xB3 // higher=increased power consumption; lower=may flicker / 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
//    sendCommand(SSD1331_CMD_MUXRATIO, 11);    // number rows to be muxed
    sendCommand(SSD_CMD_SETREMAP, INITIAL_REMAP_VALUE);
    sendCommand(SSD_CMD_STARTLINE, 0);
    sendCommand(SSD_CMD_DISPLAYOFFSET, 0x0);
    sendCommand(SSD_CMD_VCOMH, 0x09);
    //sendCommand(SSD1331_CMD_DIM_MODE_SETTING, 0, 100, 100, 100, 20);
         setRegion(0, 0, 60, 50);
         fill(50);     // clear VRam before activating display
     //    sendCommand(SSD1331_STOP_SCROLL);
      sendCommand(SSD1331_CMD_DISPLAYALLOFF);
    sendCommand(SSD1331_CMD_CONTRASTMASTER, 10);
    sendCommand(SSD_CMD_DISPLAY_ON);
//    sendCommand(SSD_CMD_DISPLAYALLON);
        sendCommand(SSD1331_CMD_CLEAR_RECT, {0, 0, 95, 63});
//    sendCommand(SSD1331_CMD_DRAW_LINE, 0, 0, 50, 50, 0, 0, 0);
//    sendCommand(SSD1331_CMD_POWERSAVE_ON, 0xB);
  }

  void fill(bool wait = true)
  {
    _dmaTransfer.reset();
    enableWrite();
    spi_set_format(_spiOut, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
#if 0 // for testing
    for(uint16_t i = 0 ; i < _regionSize ; ++i)
    {
      spi_write16_blocking(_spiOut, &_clrFill, 1);
    }
    spi_set_format(_spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
#else
    if (_regionSize == 1)
    {
      spi_write16_blocking(_spiOut, &_clrFill, 1);
      spi_set_format(_spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
      return;
    }

    if (auto chan = DMAChannel(true))
    {
      dma_channel_config cfg = { 0 };
      channel_config_set_chain_to(&cfg, chan);
      channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
      channel_config_set_enable(&cfg, true);
      channel_config_set_irq_quiet(&cfg, true);
      channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);

      dma_channel_hw_addr(chan)->al1_ctrl = channel_config_get_ctrl_value(&cfg);
      dma_channel_configure(chan, &cfg, &spi_get_hw(_spiOut)->dr, &_clrFill, _regionSize, true);

      if (wait)
      {
        dma_channel_wait_for_finish_blocking(chan);
        // when DMA finished, this doesn't mean SPI has finished, so wait for it
        _spiOut.waitTransfer();
        spi_set_format(_spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
      }
      else
      {
        _dmaTransfer.reset([this, chan = chan.release()]() // transfer ownership of chan to transfer functor
          {
            dma_channel_wait_for_finish_blocking(chan);
            // when DMA finished, this doesn't mean SPI has finished, so wait for it
            _spiOut.waitTransfer();
            spi_set_format(_spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
            dma_channel_unclaim(chan);
          });
      }
    }
#endif
  }

  void texture_mapping_setup(const uint16_t *texture, uint texture_width_bits, uint texture_height_bits, uint uv_fractional_bits) {
    texture_width_bits *= sizeof(uint16_t); // texture is 2 bytes!
//    texture_height_bits <<= 1;
    interp_config cfg = interp_default_config();
    // set add_raw flag to use raw (un-shifted and un-masked) lane accumulator value when adding
    // it to the lane base to make the lane result
    interp_config_set_add_raw(&cfg, true);
    interp_config_set_shift(&cfg, uv_fractional_bits);
    interp_config_set_mask(&cfg, 0, texture_width_bits - 1);
    interp_set_config(interp0, 0, &cfg);

    interp_config_set_shift(&cfg, uv_fractional_bits - texture_width_bits);
    interp_config_set_mask(&cfg, texture_width_bits, texture_width_bits + texture_height_bits - 1);
    interp_set_config(interp0, 1, &cfg);

    interp0->base[2] = (uintptr_t)texture;
  }

  void texture_mapped_span(uint16_t *output, uint32_t u, uint32_t v, uint32_t du, uint32_t dv, uint count) {
    // u, v are texture coordinates in fixed point with uv_fractional_bits fractional bits
    // du, dv are texture coordinate steps across the span in same fixed point.
    interp0->accum[0] = u;
    interp0->base[0] = du;
    interp0->accum[1] = v;
    interp0->base[1] = dv;
    for (uint i = 0; i < count; i++) {
      // equivalent to
      // uint32_t sm_result0 = (accum0 >> uv_fractional_bits) & (1 << (texture_width_bits - 1);
      // uint32_t sm_result1 = (accum1 >> uv_fractional_bits) & (1 << (texture_height_bits - 1);
      // uint8_t *address = texture + sm_result0 + (sm_result1 << texture_width_bits);
      // output[i] = *address;
      // accum0 = du + accum0;
      // accum1 = dv + accum1;

      // result2 is the texture address for the current pixel;
      // popping the result advances to the next iteration
      output[i] = *(uint16_t *) interp0->pop[2];
    }
  }

  void drawSprite(uint8_t x, uint8_t y, uint16_t const* data, uint8_t w, uint8_t h, int8_t scale, bool wait)
  {
    if (abs(scale) <= 1)
    {
      _dmaTransfer.reset();
      setRegion(x, y, x+w-1, y+h-1);
      if (auto chan = DMAChannel(true))
      {
        dma_channel_config cfg = { 0 };

        channel_config_set_chain_to(&cfg, chan);
        channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
        channel_config_set_enable(&cfg, true);
        channel_config_set_irq_quiet(&cfg, true);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        dma_channel_hw_addr(chan)->al1_ctrl = channel_config_get_ctrl_value(&cfg);
        dma_channel_configure(chan, &cfg, &spi_get_hw(_spiOut)->dr, data, _regionSize, true);
        auto finishTransfer = [this, chan = chan.release()]()
          {
            dma_channel_wait_for_finish_blocking(chan);
            // when DMA finished, this doesn't mean SPI has finished, so wait for it
            _spiOut.waitTransfer();
            _spiOut.setBits(8);
            dma_channel_unclaim(chan);
          };

        if (wait)
          finishTransfer();
        else
          _dmaTransfer.reset(finishTransfer);
      }
      return;
    }
    const uint ACCUM_SHIFT = 16;
    interp_claim_lane_mask(interp0, 3u);
    texture_mapping_setup(data, w, h, ACCUM_SHIFT);
    uint16_t out;
    texture_mapped_span(&out, 0, 0, (1 << ACCUM_SHIFT) / 2, (1 << ACCUM_SHIFT) / 2, 1);
    interp_unclaim_lane(interp0, 3u);
  }

  SPIOut _spiOut;
  Output _dc;
  Output _rst;
#if NEED_CS_PIN
  Output _cs;
#endif
  Transaction _dmaTransfer;
  uint16_t _regionSize;
  uint16_t _clrDraw;
  uint16_t _clrFill;

  uint8_t _x0, _x1, _y0, _y1;
  uint8_t _remap;
  uint8_t _swapCoords;
};


}; // namespace


SSD1351::SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst)
{
  static_assert(sizeof(SSD1351Priv) <= sizeof(_priv), "size of _priv too small, please increase");
  ::new(_priv) SSD1351Priv(gpio_mosi, gpio_sck, gpio_dc, gpio_rst); // placement new
}

SSD1351::~SSD1351()
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.~SSD1351Priv();
}

void SSD1351::init()
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.initDisplay();
}


void SSD1351::fillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool wait)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.setRegion(x0, y0, x1, y1);
  p.fill(wait);
}

void SSD1351::standBy(StandBy t)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  if (t == StandBy::OFF) {
    p.sendCommand(SSD1351_CMD_FUNCTIONSELECT, 0x01);
    sleep_ms(1);
    p.sendCommand(SSD_CMD_DISPLAY_ON);
    return;
  }
  p.sendCommand(SSD_CMD_DISPLAY_OFF);
  if (t == StandBy::DEEP)
    p.sendCommand(SSD1351_CMD_FUNCTIONSELECT, 0x00);
}

#if NEED_CS_PIN

Transaction SSD1351::beginTransaction()
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  p->_cs << 0;
  return Transaction([p]()
  {
    p->_cs << 1;
  });
}
#endif

void SSD1351::setStartLine(uint l)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.sendCommand(SSD_CMD_STARTLINE, l);
}

void SSD1351::horizontalScroll(uint8_t numSteps, uint8_t startRow, uint8_t numRows, ScrollSpeed spd)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.sendCommand(SSD_CMD_HORIZSCROLL, {0, 0, 0, 0, (uint8_t)spd});
}

void SSD1351::waitTransfer()
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p._dmaTransfer.reset();
}

void SSD1351::drawSprite(uint8_t x, uint8_t y, uint16_t const* data, uint8_t w, uint8_t h, int8_t scale, bool wait)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.drawSprite(x, y, data, w, h, scale, wait);
}

void SSD1351::invertDisplay(bool inverted)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.sendCommand(SSD1351_CMD_NORMALDISPLAY | inverted);
}

void SSD1351::setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.setRegion(x0, y0, x1, y1);
}

void SSD1351::setPixels(uint16_t numPixels)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  if (numPixels < 4)
  {
    while (numPixels--)
      spi_write16_blocking(p._spiOut, &p._clrDraw, 1);
    return;
  }

  if (auto chan = DMAChannel(true))
  {
    dma_channel_config cfg = { 0 };
    channel_config_set_chain_to(&cfg, chan);
    channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
    channel_config_set_enable(&cfg, true);
    channel_config_set_irq_quiet(&cfg, true);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);

    dma_channel_hw_addr(chan)->al1_ctrl = channel_config_get_ctrl_value(&cfg);
    dma_channel_configure(chan, &cfg, &spi_get_hw(p._spiOut)->dr, &p._clrDraw, p._regionSize, true);

    dma_channel_wait_for_finish_blocking(chan);
    // when DMA finished, this doesn't mean SPI has finished, so wait for it
    p._spiOut.waitTransfer();
    spi_set_format(p._spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  }
}

Transaction SSD1351::beginPixelPath()
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.enableWrite();
  p._spiOut.setBits(16);
  return Transaction([&p]()
    {
      p._spiOut.setBits(8);
    });
}

// With certain rotation changes the screen contents may change immediately into a peculiar format(mirrored,
//  not necessarily rotated). Therefore, it's recommend to clear the screen before changing rotation.
void SSD1351::setRotation(Rotation rot, bool clearScreen)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);

  if (clearScreen)
  {
    p.setRegion();
    p.fill();
  }
  uint8_t v = INITIAL_REMAP_VALUE;
  switch (rot)
  {
  case Rotation::DEGREE_0:     v |= REMAP_SCAN_DIRECTION; break; // scan bottom up
  case Rotation::DEGREE_90 :   v |= REMAP_SCAN_DIRECTION | REMAP_COL_ADDRESS | REMAP_VERTICAL_INC; break; // Scan bottom up, column remap 127-0, vertical
  case Rotation::DEGREE_180 :  v |= REMAP_COL_ADDRESS; break; // column remap 127-0
  case Rotation::DEGREE_270 :  v |= REMAP_VERTICAL_INC; break; // vertical
  }

  p.sendCommand(SSD_CMD_SETREMAP, v);
  p.sendCommand(SSD_CMD_STARTLINE, (rot < SSD1351::Rotation::DEGREE_180) * SSD1351::HEIGHT);
//  auto diffBits = p._swapCoords ^ v;
//  if (diffBits & REMAP_VERTICAL_INC)
//  {
//    swap(p._x0, p._y0);
//    swap(p._x1, p._y1);
//  }
//  if (diffBits & REMAP_SCAN_DIRECTION)
//  {
//    swap(p._x0, p._x1);
//    swap(p._y0, p._y1);
//  }
  p._swapCoords = v;
}

void SSD1351::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  if ((x0 == x1) || (y0 == y1))
  {
    p.setRegion(x0, y0, x1, y1);
    auto rFill = remember(p._clrFill, color);
    p.fill();
    return;
  }

  uint8_t dy = abs(y1 - y0);
  bool steep = dy > abs(x1 - x0);
  if (steep)
  {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1)
  {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint8_t dx = x1 - x0;

  int8_t err = dx / 2;
  int8_t ystep = (y0 < y1) ? 1 : -1;

  int16_t xbegin = x0;
  if (steep)
  {
    for (; x0 <= x1; ++x0)
    {
      err -= dy;
      if (err < 0) {
        drawLine(y0, xbegin, y0, x0, color);
        xbegin = x0 + 1;
        y0 += ystep;
        err += dx;
      }
    }
    if (x0 > xbegin + 1)
      drawLine(y0, xbegin, y0, x0, color);
  }
  else
  {
    for (; x0 <= x1; ++x0)
    {
      err -= dy;
      if (err < 0) {
        drawLine(xbegin, y0, x0, y0, color);
        xbegin = x0 + 1;
        y0 += ystep;
        err += dx;
      }
    }
    if (x0 > xbegin + 1)
      drawLine(xbegin, y0, x0, y0, color);
  }
}

void SSD1351::fill(bool wait /* = true */)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.fill(wait);
}


void SSD1351::setDrawColor(uint16_t clr)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p._clrDraw = clr;
}


void SSD1351::setFillColor(uint16_t clr)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p._clrFill = clr;
}


void SSD1351::setMirror(Mirror m, bool clearScreen /* = true */)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);

  if (clearScreen)
  {
    uint8_t x0 = p._x0, y0 = p._y0, x1 = p._x1, y1 = p._y1;
    p.setRegion();
    auto rFill = remember(p._clrFill, (uint16_t)0);
    p.fill();
    p.setRegion(x0, y0, x1, y1);
  }
  uint8_t v = INITIAL_REMAP_VALUE;
  switch (m)
  {
  case Mirror::NO:
    break;
  case Mirror::HORIZONTAL:   v |= 0b00000001; break;
    break;
  case Mirror::VERTICAL:
    break;
  case Mirror::BOTH:  v |= 0b00000010;  break;
  }
//  p._swapCoords = false;
  p.sendCommand(SSD_CMD_SETREMAP, v);
  p.sendCommand(SSD_CMD_STARTLINE, (m < Mirror::VERTICAL) * SSD1351::HEIGHT);
}


void SSD1351::setAngle(uint8_t angle)
{
  // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
  // coords (bits 16:9 of addr offset), and we'll represent coords with
  // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
  // contain increment vector, and BASE2 will contain image base pointer

#define UNIT_LSB 16
#define LOG2_DISPLAY_SIZE 7
  interp_config lane0_cfg = interp_default_config();
  interp_config_set_shift(&lane0_cfg, UNIT_LSB - 1);   // -1 because 2 bytes per pixel
  interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG2_DISPLAY_SIZE - 1));
  interp_config_set_add_raw(&lane0_cfg, true);   // Add full accumulator to base with each POP
  interp_config lane1_cfg = interp_default_config();
  interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG2_DISPLAY_SIZE));
  interp_config_set_mask(&lane1_cfg, 1 + LOG2_DISPLAY_SIZE, 1 + (2 * LOG2_DISPLAY_SIZE - 1));
  interp_config_set_add_raw(&lane1_cfg, true);

  interp_set_config(interp0, 0, &lane0_cfg);
  interp_set_config(interp0, 1, &lane1_cfg);
  //  interp0->base[2] = (uint32_t) raspberry_256x256;

//  float theta = 0.f;
//  float theta_max = 2.f * (float) M_PI;
//  while (1) {
//    theta += 0.02f;
//    if (theta > theta_max)
//      theta -= theta_max;
//    int32_t rotate[4] = {
//      cosf(theta) * (1 << UNIT_LSB),
//      -sinf(theta) * (1 << UNIT_LSB),
//      sinf(theta) * (1 << UNIT_LSB),
//      cosf(theta) * (1 << UNIT_LSB)
//    };
//    interp0->base[0] = rotate[0];
//    interp0->base[1] = rotate[2];
//    st7789_start_pixels(pio, sm);
//    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
//      interp0->accum[0] = rotate[1] * y;
//      interp0->accum[1] = rotate[3] * y;
//      for (int x = 0; x < SCREEN_WIDTH; ++x) {
//        uint16_t colour = *(uint16_t *)(interp0->pop[2]);
//      }
//    }
//  }
}