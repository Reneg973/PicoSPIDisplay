#pragma once

#include "hardware/gpio.h"
#include <type_traits>

class Bus;

class GPIOMask
{
protected:
  GPIOMask() = delete;

  explicit GPIOMask(uint gpio)
    : _gpio_mask(1ul << gpio)
  {
  }

  GPIOMask& operator=(GPIOMask const& other)
  {
    _gpio_mask = other._gpio_mask;
  }

  void setDirection(bool bOut)
  {
    if (bOut)
      gpio_set_dir_out_masked(_gpio_mask);
    else
      gpio_set_dir_in_masked(_gpio_mask);
  }

  operator uint() const
  {
    return _gpio_mask;
  }

  uint32_t _gpio_mask;
  friend class Bus;
};

class Output : protected GPIOMask
{
public:
  explicit Output(uint gpio)
    : GPIOMask(gpio)
  {
    gpio_init(gpio);
    setDirection(true);
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
    setDirection(false);
  }

  operator bool() const
  {
    return _gpio_mask & sio_hw->gpio_in;
  }
};

class Bus : public GPIOMask
{
public:
  Bus() = delete;

  template<typename... Ts>
    Bus(bool isOutput, std::enable_if_t<std::is_same_v<Ts, uint>, Ts> const&... ts)
      : GPIOMask(0)
  {
    uint const res[sizeof...(ts)] = { ts... };
    uint mask = 0;
    for (auto& r : res)
    {
      mask |= 1u << r;
      gpio_init(r);
    }
    _gpio_mask = mask;
    setDirection(isOutput);
  }

  Bus& operator <<(uint32_t v)
  {
    return operator=(v);
  }

  Bus& operator =(uint32_t v)
  {
    gpio_put_masked(_gpio_mask, v);
    return *this;
  }

  uint read() const
  {
    return gpio_get_all();
  }

  operator uint() const
  {
    return read();
  }
};
