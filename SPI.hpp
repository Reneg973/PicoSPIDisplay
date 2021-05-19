#pragma once

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <initializer_list>

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

  inline void setBits(uint8_t data_bits)
  {
    spi_set_format(*this, data_bits, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
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

  void write(std::initializer_list<uint8_t> const& data)
  {
    spi_write_blocking(*this, data.begin(), data.size());
  }

  void waitTransfer() const
  {
    while (!spi_is_writable(*this))
      tight_loop_contents();
    while (spi_is_busy(*this))
      tight_loop_contents();
    while (spi_is_readable(*this))
      spi_get_hw(*this)->dr;
  }

private:
  uint8_t _mosi;
  uint8_t _sck;
};
