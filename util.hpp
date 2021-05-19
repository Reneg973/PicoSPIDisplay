#pragma once
#include "pico/stdlib.h"
#include <functional>

template<typename T>
_GLIBCXX14_CONSTEXPR inline void swap(T& a, T& b) { a ^= b; b ^= a; a ^= b; }

template<typename T>
_GLIBCXX14_CONSTEXPR inline void inplace_minmax(T& a, T& b)
{
  // concept requirements
  __glibcxx_function_requires(_LessThanComparableConcept<T>)

  if(b < a)
    swap(a, b);
}

template<typename T>
class Remember
{
public:
  Remember(T& t, const T& newVal)
    : m_cache(t)
    , m_t(t)
  {
    t = newVal;
  }
  ~Remember()
  {
    m_t = m_cache;
  }

private:
  T& m_t;
  const T m_cache;
};

template<typename T, typename U, typename = std::enable_if_t<std::is_convertible_v<T, U>>>
constexpr decltype(auto) remember(T& val, U const& newVal)
{
  return Remember<T>(val, static_cast<T>(newVal));
}
