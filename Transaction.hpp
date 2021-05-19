#pragma once

#include <functional>

class Transaction
{
public:
  Transaction(std::function<void()> endFunc = NULL)
  {
    _endFunc.swap(endFunc);
  }

  Transaction(const Transaction&) = delete;
  Transaction& operator=(Transaction&) = delete;

  ~Transaction()
  {
    reset();
  }

  void reset(std::function<void()> endFunc = NULL)
  {
    if (_endFunc)
      _endFunc();
    _endFunc.swap(endFunc);
  }

  operator bool() const
  {
    return (bool)_endFunc;
  }

private:
  std::function<void()> _endFunc;
};

