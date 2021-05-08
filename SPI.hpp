#pragma once

#include "hardware/gpio.h"
#include "hardware/spi.h"

#if 0
class SPIInst
{
public:
  SPIInst(spi_inst_t * spi)
    : _spi(spi)
  {
  }
  ~SPIInst()
  {
  }
private:
  spi_inst_t* _spi;
};
#endif

class SPIOut
{
public:
  SPIOut(uint gpio_mosi, uint gpio_sck, uint bauds, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha)
    : _mosi(gpio_mosi)
    , _sck(gpio_sck)
  {
    gpio_set_function(_mosi, GPIO_FUNC_SPI);
    gpio_set_function(_sck, GPIO_FUNC_SPI); 

    spi_init(*this, bauds, data_bits, cpol, cpha);
  }
  
  ~SPIOut()
  {
    spi_deinit(*this);
    gpio_set_function(_mosi, GPIO_FUNC_NULL);
    gpio_set_function(_sck, GPIO_FUNC_NULL);
  }

  inline uint set_baudrate(uint bauds)
  {
    return spi_set_baudrate(*this, bauds);
  }
  
  inline operator spi_inst_t*() const
  {
    return spi0;  // TODO: check spi via gpio_mosi and gpio_sck
  }

  inline void write(uint8_t v)
  {
    spi_write_blocking(*this, &v, sizeof(v));
  }

  void write(uint8_t v1, uint8_t v2)
  {
    uint8_t v[] { v1, v2 };
    spi_write_blocking(*this, v, sizeof(v));
  }
  
  void write(uint8_t v1, uint8_t v2, uint8_t v3)
  {
    uint8_t v[] { v1, v2, v3 };
    spi_write_blocking(*this, v, sizeof(v));
  }

  void write(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4)
  {
    uint8_t v[] { v1, v2, v3, v4 };
    spi_write_blocking(*this, v, sizeof(v));
  }
  
  void write(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5, uint8_t v6, uint8_t v7)
  {
    uint8_t v[] { v1, v2, v3, v4, v5, v6, v7 };
    spi_write_blocking(*this, v, sizeof(v));
  }

  void waitTransfer() const
  {
    while (!spi_is_writable(*this))
      tight_loop_contents();
    while (spi_is_busy(*this))
      tight_loop_contents();
    while (spi_is_readable(*this))
      (void)spi_get_hw(*this)->dr;
  }
  
private:
  uint8_t _mosi;
  uint8_t _sck;
};