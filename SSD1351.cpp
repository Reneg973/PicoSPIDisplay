#include "SSD1351.h"

#include "hardware/spi.h"
#include "hardware/dma.h"

#define SPI_MOSI_DISPLAY  19
#define SPI_SCK_DISPLAY   18
#define SPI_DC_DISPLAY    17
#define SPI_RST_DISPLAY   16

// SSD1351D Regs
#define SSD1351_CMD_SETCOLUMN       0x15 // 2Byte: Set start and end column of active video RAM area
#define SSD1351_CMD_WRITERAM        0x5C
#define SSD1351_CMD_READRAM         0x5D
#define SSD1351_CMD_SETROW          0x75 // 2Byte: Set start and end row of active video RAM area
#define SSD1351_CMD_HORIZSCROLL     0x96 // 5 byte command; A=# of scroll; B=start row addr; C=num rows to scroll; E=scroll interval: 1=normal; 3=slowest
#define SSD1351_CMD_STOPSCROLL      0x9E
#define SSD1351_CMD_STARTSCROLL     0x9F
#define SSD1351_CMD_SETREMAP        0xA0 // Remap various display settings, like hardware mapping and most importantly color mode
// A[0]: Horizontal mirror; A[1]: Horz or Vert increment; A[2]: Swap color sequence (A->B->C : C->B->A)
//A[4]: COM Scan direction; A5]: COM Split Odd/Even; 
#define SSD1351_CMD_STARTLINE       0xA1 // Set display start line, needs to be set to 96 for 128x96 displays
#define SSD1351_CMD_DISPLAYOFFSET   0xA2 // Set display offset (hardware dependent, needs to be set to 0; locked)
#define SSD1351_CMD_DISPLAYALLOFF   0xA4
#define SSD1351_CMD_DISPLAYALLON    0xA5
#define SSD1351_CMD_NORMALDISPLAY   0xA6
#define SSD1351_CMD_INVERTDISPLAY   0xA7
#define SSD1351_CMD_FUNCTIONSELECT  0xAB
#define SSD1351_CMD_DISPLAYOFF      0xAE
#define SSD1351_CMD_DISPLAYON       0xAF
#define SSD1351_CMD_PRECHARGE       0xB1
#define SSD1351_CMD_DISPLAYENHANCE  0xB2 // 3Byte command; 0:0:0 = normal; A4:0:0 = enhance display performance
#define SSD1351_CMD_CLOCKDIV        0xB3
#define SSD1351_CMD_SETVSL          0xB4 // 3Byte: A0 = internal VSL; B+C constant
#define SSD1351_CMD_SETGPIO         0xB5
#define SSD1351_CMD_PRECHARGE2      0xB6
#define SSD1351_CMD_SETGRAY         0xB8 //when setting this, a list of grayscale table entries must follow (GS1..GS63)
#define SSD1351_CMD_USELUT          0xB9 //reset gray lut set by SETGRAY
#define SSD1351_CMD_PRECHARGELEVEL  0xBB
#define SSD1351_CMD_VCOMH           0xBE
#define SSD1351_CMD_CONTRASTABC     0xC1
#define SSD1351_CMD_CONTRASTMASTER  0xC7 // reduce current output; 0:1/16 to F:16/16
#define SSD1351_CMD_MUXRATIO        0xCA
#define SSD1351_CMD_COMMANDLOCK     0xFD

#define swap(a, b) { auto t = a; a = b; b = a; }

namespace
{
struct SSD1351Priv
{
  void writeCommand(uint8_t cmd)
  {
    gpio_put(_dc, false);
    spi_write_blocking(spi0, &cmd, sizeof(cmd));
  }

  void writeCommand(uint8_t cmd, uint8_t d1)
  {
    writeCommand(cmd);
    gpio_put(_dc, true);
    spi_write_blocking(spi0, &d1, sizeof(d1)); 
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2)
  {
    writeCommand(cmd);
    gpio_put(_dc, true);
    uint8_t v[] {d1, d2};
    spi_write_blocking(spi0, v, sizeof(v)); 
  }

  void writeCommand(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3)
  {
    writeCommand(cmd);
    gpio_put(_dc, true);
    uint8_t v[] {d1, d2, d3};
    spi_write_blocking(spi0, v, sizeof(v)); 
  }

  //Enable writing to the SSD1351 RAM
  void enableWrite() {
    writeCommand(SSD1351_CMD_WRITERAM);
    gpio_put(_dc, true);
  }

  void setRegion(uint8_t x0 = 0, uint8_t y0 = 0, uint8_t x1 = 127, uint8_t y1 = 127)
  {
    if (x0 > x1)
      swap(x0, x1);
    if (y0 > y1)
      swap(y0, y1);
    writeCommand(SSD1351_CMD_SETROW, y0, y1);
    writeCommand(SSD1351_CMD_SETCOLUMN, x0, x1);
    _regionSize = ((uint16_t)x1 - x0 + 1) * ((uint16_t)y1 - y0 + 1);
  }

  void initDisplay()
  {
#define STABLE_RESET_TIME_US  50 // even if datasheet says 2us, this is not stable when resetting
    gpio_put(_rst, true);
    sleep_us(STABLE_RESET_TIME_US);
    gpio_put(_rst, false);
    sleep_us(STABLE_RESET_TIME_US);
    gpio_put(_rst, true);
    sleep_us(STABLE_RESET_TIME_US);
#define COMMAND_LOCK_INIT1 0x12 // "Unlock OLED driver IC MCU interface from entering command"
    writeCommand(SSD1351_CMD_COMMANDLOCK, COMMAND_LOCK_INIT1);     // set command lock
#undef COMMAND_LOCK_INIT1
#define COMMAND_LOCK_INIT2 0xB1 // "Command A2,B1,B3,BB,BE accessible if in unlock state"  
    writeCommand(SSD1351_CMD_COMMANDLOCK, COMMAND_LOCK_INIT2);      // set command lock
#undef COMMAND_LOCK_INIT2
    writeCommand(SSD1351_CMD_DISPLAYOFF);          // 0xAE

    writeCommand(SSD1351_CMD_CLOCKDIV, 0x2F);      // 0xB3 // higher=increased power consumption; lower=may flicker / 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    writeCommand(SSD1351_CMD_MUXRATIO, 127);    // number rows to be muxed
    
    writeCommand(SSD1351_CMD_SETREMAP, 0x74);    //0x74
  
    setRegion();

    writeCommand(SSD1351_CMD_STARTLINE, 0);

    writeCommand(SSD1351_CMD_DISPLAYOFFSET, 0x0);

    writeCommand(SSD1351_CMD_FUNCTIONSELECT, 0x01);    // internal (diode drop)

    writeCommand(SSD1351_CMD_PRECHARGE, 0x32);
    writeCommand(SSD1351_CMD_VCOMH, 0x05);
    writeCommand(SSD1351_CMD_NORMALDISPLAY);
    writeCommand(SSD1351_CMD_CONTRASTABC, 0xC8, 0xC8, 0xC8);
    writeCommand(SSD1351_CMD_CONTRASTMASTER, 2);
    writeCommand(SSD1351_CMD_SETVSL, 0xA0, 0xB5, 0x55);
    writeCommand(SSD1351_CMD_PRECHARGE2, 0x01);

    writeCommand(SSD1351_CMD_DISPLAYON);
  }

  void fill(uint16_t color)
  {
    enableWrite();
#if 0
#if REMEMBER_DMA

    if (_dmaChan != -1)
    {
      dma_channel_configure(_dmaChan,
        &_dmaCfg,
        &spi_get_hw(_spi)->dr,
          // The initial write address
        &color,
                         // The initial read address
        _regionSize*2,
                  // Number of transfers
        true                    // Start immediately.
      );
      dma_channel_wait_for_finish_blocking(_dmaChan);
    }
    else
#endif
    {
      for (uint16_t i = 0; i < _regionSize; ++i)
      {
        spi_write_no_read_blocking(_spi, (uint8_t*)&color, 2);
      }
    }
#else
    bool bDMA = false;
    if(_regionSize >= 8)
      if (int chan = dma_claim_unused_channel(true); chan != -1)
      {
        auto cfg = dma_channel_get_default_config(chan);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_ring(&cfg, false, 1);
        channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
        channel_config_set_bswap(&cfg, true);
        dma_channel_set_config(chan, &cfg, false);
        dma_channel_configure(chan, &cfg, &spi_get_hw(_spi)->dr, &color, _regionSize * 2, true);
        dma_channel_wait_for_finish_blocking(chan);
        dma_channel_wait_for_finish_blocking(chan);
        dma_channel_unclaim(chan);
        bDMA = true;
      }
    if(!bDMA)
    {
      for (uint16_t i = 0; i < _regionSize; ++i)
      {
        spi_write_no_read_blocking(_spi, (uint8_t*)&color, 2);
      }
    }
#endif
  }
  spi_inst_t* _spi;
  uint16_t _regionSize;
  uint8_t _mosi;
  uint8_t _sck;
  uint8_t _dc;
  uint8_t _rst;
  uint8_t _cs;
#if REMEMBER_DMA
  int _dmaChan;
  dma_channel_config _dmaCfg;
#endif
};

		 
}; // namespace


SSD1351::SSD1351(uint gpio_mosi, uint gpio_sck, uint gpio_dc, uint gpio_rst, uint gpio_cs/* = 0*/)
{
  static_assert(sizeof(SSD1351Priv) <= sizeof(_priv), "size of _prive too small, please increase");
  
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  p->_mosi = gpio_mosi;
  p->_sck = gpio_sck;
  p->_dc = gpio_dc;
  p->_rst = gpio_rst;
  p->_cs = gpio_cs;
  p->_spi = spi0; // TODO: check spi via gpio_mosi and gpio_sck
#if REMEMBER_DMA
  p->_dmaChan = -1;
#endif
  
  gpio_init_mask((1 << gpio_dc) | (1 << gpio_rst) | (1 << gpio_cs));
  gpio_set_dir_out_masked((1 << gpio_dc) | (1 << gpio_rst) | (1 << gpio_cs));
  spi_init(p->_spi, 40 * 1000 * 1000);
  spi_set_format(p->_spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(gpio_mosi, GPIO_FUNC_SPI);
  gpio_set_function(gpio_sck, GPIO_FUNC_SPI);
  
  p->initDisplay();

  p->setRegion(0, 0, 127, 127);
  p->enableWrite();
  uint16_t color = 0;
  for (uint16_t i = 0; i < p->_regionSize; ++i)
  {
    spi_write_no_read_blocking(p->_spi, (uint8_t*)&color, 2);
  }
}

SSD1351::~SSD1351()
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  spi_deinit(p->_spi);
  gpio_set_function(p->_mosi, GPIO_FUNC_NULL);
  gpio_set_function(p->_sck, GPIO_FUNC_NULL);
#if REMEMBER_DMA
  uninitDma();
#endif
}


void SSD1351::fillRect(uint x0, uint y0, uint x1, uint y1, uint16_t color)
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  p->setRegion(x0, y0, x1, y1);
  p->fill(color);
}

void SSD1351::standBy(StandByType t)
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  if (t == StandByType::Off)
    p->writeCommand(SSD1351_CMD_DISPLAYON);
  else if(t==StandByType::Weak)
    p->writeCommand(SSD1351_CMD_DISPLAYOFF);
}

#if REMEMBER_DMA

bool SSD1351::initDma()
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  if (p->_dmaChan != -1)
    return true;
  
  int chan = dma_claim_unused_channel(true);
  if (chan == -1)
    return false;

  p->_dmaChan = chan;
  auto cfg = dma_channel_get_default_config(chan);
  channel_config_set_transfer_data_size(&p->_dmaCfg, DMA_SIZE_8);
  channel_config_set_read_increment(&p->_dmaCfg, true);
  channel_config_set_write_increment(&p->_dmaCfg, false);
  channel_config_set_ring(&p->_dmaCfg, false, 1);
  channel_config_set_dreq(&p->_dmaCfg, DREQ_SPI0_TX);
  channel_config_set_bswap(&p->_dmaCfg, true);
  dma_channel_set_config(chan, &p->_dmaCfg, false);
  p->_dmaCfg = cfg;
}

void SSD1351::uninitDma()
{
  auto p = reinterpret_cast<SSD1351Priv*>(_priv);
  if (p->_dmaChan == -1)
    return;
  dma_channel_wait_for_finish_blocking(p->_dmaChan);
  dma_channel_unclaim(p->_dmaChan);
  p->_dmaChan = -1;
}
#endif