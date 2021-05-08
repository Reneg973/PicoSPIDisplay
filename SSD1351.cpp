#include "hardware/spi.h"
#include "hardware/dma.h"
#include <stdio.h>
#include <new>

#include "SSD1351.h"
#include "gpio.hpp"
#include "SPI.hpp"

// SSD1351D Regs
#define SSD_CMD_SETCOLUMN           0x15 // 2Byte: Set start and end column of active video RAM area
#define SSD1331_CMD_DRAW_LINE       0x21 // 7 bytes: A: col0; B: row0; C: col1; D: row1; E: color C; F: color B; G: color A
#define SSD1331_CMD_FILL_RECT       0x22 // 10 bytes: A: col0; B: row0; C: col1; D: row1; E..G: linecolor; H..J: fillcolor
#define SSD1331_CMD_COPY_RECT       0x23 // 6 bytes: A: col0; B: row0; C: col1; D: row1; E: colNew; F: rowNew
#define SSD1331_CMD_DIM_RECT        0x24 // 4 bytes: A: col0; B: row0; C: col1; D: row1; reduces intensity of given area
#define SSD1331_CMD_CLEAR_RECT      0x25 // 4 bytes: A: col0; B: row0; C: col1; D: row1
#define SSD1331_CMD_ENABLEFILL_RECT 0x26 // 0: default unfilled; 1: use fill color for fill rect cmd
#define SSD1331_CMD_CONTINUOUS_SCROLL_SETUP 0x27 // 5 bytes; A=horiz. scroll offset columns; B: start row; C: numRowsToScroll; D: vert. scroll offset; E: interval 0..3
#define SSD1331_STOP_SCROLL         0x2E
#define SSD1331_START_SCROLL        0x2F
#define SSD1351_CMD_WRITERAM        0x5C
#define SSD1351_CMD_READRAM         0x5D
#define SSD_CMD_SETROW              0x75 // 2Byte: Set start and end row of active video RAM area
#define SSD1331_SET_CONTRAST_A      0x81
#define SSD1331_SET_CONTRAST_B      0x82
#define SSD1331_SET_CONTRAST_C      0x83
#define SSD1331_CMD_CONTRASTMASTER  0x87 // see SSD1351_CMD_CONTRASTMASTER
#define SSD1331_CMD_PRECHARGE2
#define SSD_CMD_HORIZSCROLL         0x96 // 5 byte command; A=# of scroll; B=start row addr; C=num rows to scroll; E=scroll interval: 1=normal; 3=slowest
#define SSD_CMD_STOPSCROLL          0x9E
#define SSD_CMD_STARTSCROLL         0x9F
#define SSD_CMD_SETREMAP            0xA0 // Remap various display settings, like hardware mapping and most importantly color mode
// A[0]: Horizontal or vertical increment; A[1]: map column address to 0 or 127; A[2]: Swap color sequence (A->B->C : C->B->A)
//A[4]: COM Scan direction; A5]: COM Split Odd/Even; A[7:6] color depth=65k; 262k fmt1; 262k fmt2 
#define SSD_CMD_STARTLINE           0xA1 // Set display start line, needs to be set to 96 for 128x96 displays
#define SSD_CMD_DISPLAYOFFSET       0xA2 // Set display offset (hardware dependent, needs to be set to 0; locked)
#define SSD1351_CMD_DISPLAYALLOFF   0xA4
#define SSD1331_CMD_NORMALDISPLAY   0xA4
#define SSD_CMD_DISPLAYALLON        0xA5
#define SSD1351_CMD_NORMALDISPLAY   0xA6
#define SSD1331_CMD_DISPLAYALLOFF   0xA6
#define SSD_CMD_INVERSEDISPLAY      0xA7
#define SSD1331_CMD_DIM_MODE_SETTING  0xAB // 5 bytes: A: reserved (0); B: contrast A; C: contrast B; D: contrast C; E: precharge range 0..31
#define SSD1351_CMD_FUNCTIONSELECT  0xAB // A[0]: Disable internal Vdd (power save)
#define SSD1331_CMD_DISPLAY_DIM     0xAC
#define SSD_CMD_DISPLAY_OFF         0xAE
#define SSD_CMD_DISPLAY_ON          0xAF
#define SSD1331_CMD_POWERSAVE_ON    0xB0  // default = 1A (enabled); disabled = 0B
#define SSD1331_CMD_PHASE_PERIOD_ADJUSTMENT       0xB1
#define SSD1351_CMD_PRECHARGE       0xB1  // locked by default
#define SSD1351_CMD_DISPLAYENHANCE  0xB2 // 3Byte command; 0:0:0 = normal; A4:0:0 = enhance display performance
#define SSD_CMD_CLOCKDIV            0xB3 // A[7:4]: freq; A[3:0] Div
#define SSD1351_CMD_SETVSL          0xB4 // 3Byte: A0 = internal VSL; B+C constant
#define SSD1351_CMD_SETGPIO         0xB5 // no usage on SPI display
#define SSD1351_CMD_PRECHARGE2      0xB6 // 0 = invalid: 1=1DCLK; 2=2DCLK; F=15DCLK
#define SSD_CMD_SETGRAY             0xB8 //when setting this, a list of grayscale table entries must follow (GS1..GS63)
#define SSD_CMD_USEBUILTIN_LUT      0xB9 //reset gray lut set by SETGRAY to linear
#define SSD_CMD_PRECHARGELEVEL      0xBB // locked
#define SSD_CMD_VCOMH               0xBE // locked
#define SSD1351_CMD_CONTRASTABC     0xC1 // 3 bytes
#define SSD1351_CMD_CONTRASTMASTER  0xC7 // reduce current output; 0:1/16 to F:16/16
#define SSD1351_CMD_MUXRATIO        0xCA
#define SSD_CMD_COMMANDLOCK         0xFD // SSD1351 has more bits defined; see datasheet

template<typename T>
constexpr inline void swap(T& a, T& b) { auto t = a; a = b; b = a; }

class DMAChannel
{
public:
  explicit DMAChannel(bool bRequired=true)
  {
    _chan = dma_claim_unused_channel(bRequired);
  }
  ~DMAChannel()
  {
    dma_channel_unclaim(_chan);
  }
  
  inline operator uint() const
  {
    return _chan;
  }
    
  inline operator bool() const
  {
    return _chan != (uint8_t)-1;
  }
private:
  uint8_t _chan;
};
namespace
{

#define SPI_BAUDRATE 5 * 1000 * 1000
struct SSD1351Priv
{
  SSD1351Priv(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst
#if HAS_CS_PIN
, uint gpio_cs
#endif
    )
    : _dc(gpio_dc)
    , _spiOut(gpio_mosi, gpio_sck, SPI_BAUDRATE, 8, SPI_CPOL_1, SPI_CPHA_1)
    , _rst(gpio_rst)
#if HAS_CS_PIN
    , _cs(gpio_cs)
#endif
  {
  }
  
  void writeCommand(uint8_t cmd)
  {
    _dc << false;
    _spiOut.write(cmd);
  }

  void writeCommand(uint8_t cmd, uint8_t d1)
  {
    writeCommand(cmd);
    _dc << true;
    _spiOut.write(d1);
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2)
  {
    writeCommand(cmd);
    _dc << true;
    _spiOut.write(d1, d2);
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3)
  {
    writeCommand(cmd);
    _dc << true;
    _spiOut.write(d1, d2, d3);
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
  {
    writeCommand(cmd);
    _dc << true;
    _spiOut.write(d1, d2, d3, d4);
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
  {
    writeCommand(cmd);
    _dc << true;
    _spiOut.write(d1, d2, d3, d4, d5, d6, d7);
  }

  //Enable writing to the SSD1351 RAM
  void enableWrite()
  {
    writeCommand(SSD1351_CMD_WRITERAM);
    _dc << true;
  }

  void setRegion(uint8_t x0 = 0, uint8_t y0 = 0, uint8_t x1 = 127, uint8_t y1 = 127)
  {
    if (x0 > x1)
      swap(x0, x1);
    if (y0 > y1)
      swap(y0, y1);
    if (x0 != _x0 || x1 != _x1 || y0 != _y0 || y1 != _y1)
    {
      writeCommand(SSD_CMD_SETROW, y0, y1);
      writeCommand(SSD_CMD_SETCOLUMN, x0, x1);
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
#define COMMAND_LOCK_INIT2 0xB1 // "Command A2,B1,B3,BB,BE accessible if in unlock state"  
#else
#define COMMAND_LOCK_INIT1 0x16
#define COMMAND_LOCK_INIT2 0x12
#endif
    writeCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT1);     // set command lock
    writeCommand(SSD_CMD_COMMANDLOCK, COMMAND_LOCK_INIT2);     // set command lock
#undef COMMAND_LOCK_INIT1
#undef COMMAND_LOCK_INIT2
    writeCommand(SSD_CMD_DISPLAY_OFF);          // 0xAE

    writeCommand(SSD_CMD_CLOCKDIV, 0x2F);      // 0xB3 // higher=increased power consumption; lower=may flicker / 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    writeCommand(SSD1351_CMD_MUXRATIO, 127);    // number rows to be muxed
//    writeCommand(SSD1351_CMD_MUXRATIO, 63);    // number rows to be muxed
    writeCommand(SSD_CMD_SETREMAP, 0b00100000);    //RGB format
    writeCommand(SSD_CMD_STARTLINE, 0);
    writeCommand(SSD_CMD_DISPLAYOFFSET, 0x0);
    writeCommand(SSD1351_CMD_FUNCTIONSELECT, 0x01);    // internal (diode drop)
    writeCommand(SSD1351_CMD_PRECHARGE, 0x32);
    writeCommand(SSD_CMD_VCOMH, 0x05);
    writeCommand(SSD1351_CMD_NORMALDISPLAY);
    writeCommand(SSD1351_CMD_CONTRASTABC, 0xC8, 0xC8, 0xC8);
    writeCommand(SSD1351_CMD_CONTRASTMASTER, 7);
    writeCommand(SSD1351_CMD_SETVSL, 0xA0, 0xB5, 0x55);
    writeCommand(SSD1351_CMD_PRECHARGE2, 0x02);

    setRegion();
    fill(0);    // clear VRam before activating display

    writeCommand(SSD_CMD_DISPLAY_ON);
//    writeCommand(SSD1331_CMD_CLEAR_RECT, 0, 0, 50, 50);
//    writeCommand(SSD1331_CMD_DRAW_LINE, 0, 0, 50, 50, 0, 0, 0);
//    writeCommand(SSD1331_CMD_POWERSAVE_ON, 0xB);
  }

  void fill(uint16_t color, bool wait=true)
  {
    enableWrite();
    spi_set_format(_spiOut, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
#if 0
    {
      //color = (color & 0xFF) << 8 | (color >> 8);
      for(uint16_t i = 0 ; i < _regionSize ; ++i)
      {
        spi_write16_blocking(_spiOut, &color, 1);
      }
    }
#else
    if (_regionSize == 1)
      spi_write16_blocking(_spiOut, &color, 1);
    else if (auto chan = DMAChannel(true))
    {
      dma_channel_config cfg = { 0 };
      channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
      channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
      channel_config_set_enable(&cfg, true);
      channel_config_set_chain_to(&cfg, chan);
      dma_channel_hw_addr(chan)->al1_ctrl = channel_config_get_ctrl_value(&cfg);
      dma_channel_configure(chan, &cfg, &spi_get_hw(_spiOut)->dr, &color, _regionSize, true);
      if (wait)
      {
        dma_channel_wait_for_finish_blocking(chan);
        // when DMA finished, this doesn't mean SPI has finished, so wait for it
        _spiOut.waitTransfer();
      }
    }
#endif
    spi_set_format(_spiOut, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  }
  
  SPIOut _spiOut;
  Output _dc;
  Output _rst;
#if HAS_CS_PIN
  Output _cs;
#endif
  uint16_t _regionSize;
  uint8_t _x0, _x1, _y0, _y1;
};

		 
}; // namespace


SSD1351::SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst)
{
  static_assert(sizeof(SSD1351Priv) <= sizeof(_priv), "size of _prive too small, please increase");
  new(_priv) SSD1351Priv(gpio_mosi, gpio_sck, gpio_dc, gpio_rst); // call constructor
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


void SSD1351::fillRect(uint x0, uint y0, uint x1, uint y1, uint16_t color, bool wait)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.setRegion(x0, y0, x1, y1);
  p.fill(color, wait);
}

void SSD1351::standBy(StandByType t)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  if (t == StandByType::Off)
    p.writeCommand(SSD_CMD_DISPLAY_ON);
  else if(t==StandByType::Weak)
    p.writeCommand(SSD_CMD_DISPLAY_OFF);
}

#if HAS_CS_PIN

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

void SSD1351::setStartLine(uint8_t l)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.writeCommand(SSD_CMD_STARTLINE, l);
}

void SSD1351::horizontalScroll(uint8_t numSteps, uint8_t startRow, uint8_t numRows, ScrollSpeed spd)
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p.writeCommand(SSD_CMD_HORIZSCROLL);  
}


void SSD1351::waitTransfer()
{
  auto& p = *reinterpret_cast<SSD1351Priv*>(_priv);
  p._spiOut.waitTransfer();
}
