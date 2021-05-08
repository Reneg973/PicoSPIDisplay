#pragma once

#include "hardware/gpio.h"

class GPIOMask
{
protected:
  explicit GPIOMask(uint gpio)
    : _gpio_mask(1ul << gpio)
  {
  }
  uint const _gpio_mask;
};

class Output : protected GPIOMask
{
public:
  explicit Output(uint gpio)
    : GPIOMask(gpio)
  {
    gpio_init(gpio);
    gpio_set_dir_out_masked(_gpio_mask);
  }

  inline void operator<<(bool v) const
  {
    v ? gpio_set_mask(_gpio_mask) : gpio_clr_mask(_gpio_mask);
  }
};

class Input : protected GPIOMask
{
public:
  explicit Input(uint gpio)
    : GPIOMask(gpio)
  {
    gpio_init(gpio);
    gpio_set_dir_in_masked(_gpio_mask);
  }

  operator bool() const
  {
    return _gpio_mask & sio_hw->gpio_in;
  }
};
