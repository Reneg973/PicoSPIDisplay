#pragma once
#include "hardware/dma.h"

class DMAChannel
{
public:
  DMAChannel() = delete;
  DMAChannel(const DMAChannel&) = delete;
  DMAChannel& operator=(DMAChannel&) = delete;

  explicit DMAChannel(bool bRequired = true)
  {
    _chan = dma_claim_unused_channel(bRequired);
  }

  DMAChannel(DMAChannel&& dmaChannel)
  {
    _chan = dmaChannel._chan;
    dmaChannel._chan = 0xFF;
  }

  DMAChannel& operator=(DMAChannel&& dmaChannel)
  {
    if (_chan != 0xFF)
      dma_channel_unclaim(_chan);
    _chan = dmaChannel._chan;
    dmaChannel._chan = 0xFF;
    return *this;
  }

  ~DMAChannel()
  {
    reset();
  }

  inline uint reset()
  {
    uint c = _chan;
    if (_chan != 0xFF)
    {
      dma_channel_unclaim(c);
      _chan = 0xFF;
    }
    return c;
  }

  inline uint release()
  {
    uint c = _chan;
    if (_chan != 0xFF)
    {
      _chan = 0xFF;
    }
    return c;
  }

  constexpr inline operator uint() const
  {
    return _chan;
  }

  constexpr inline operator bool() const
  {
    return _chan != 0xFF;
  }

private:
  uint8_t _chan;
};
