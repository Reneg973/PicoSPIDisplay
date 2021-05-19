#include "hardware/gpio.h"
#include "hardware/rtc.h"
#include "hardware/timer.h"
#include "pico/time.h"

namespace
{
  int64_t heartbeatTimerAlarm(alarm_id_t id, void *user_data)
  {
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    return 0;
  }

#if USE_RTC
  void OnRTCTick()
  {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    add_alarm_in_ms(10, &heartbeatTimerAlarm, NULL, false);
  }
#else
  bool heartbeat_5s(struct repeating_timer *t) {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    add_alarm_in_ms(1, &heartbeatTimerAlarm, NULL, false);
    return true;
  }
#endif
  struct repeating_timer timer;
}

void initHeartBeat()
{
#ifndef PICO_DEFAULT_LED_PIN
#warning PICO_DEFAULT_LED_PIN not defined
#else
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#if USE_RTC
  // Start the RTC
  rtc_init();
  datetime_t t = {
    .year = 2020,
    .month = 01,
    .day = 13,
    .dotw = 3, 
    // 0 is Sunday, so 3 is Wednesday
.hour = 11,
    .min = 20,
    .sec = 00
  };

  rtc_set_datetime(&t);
  sleep_ms(20);
  constexpr datetime_t rtcAlarm {-1, -1, -1, -1, -1, -1, -5 }
  ;
  rtc_set_alarm(&rtcAlarm, &OnRTCTick);
#else
  add_repeating_timer_ms(5000, heartbeat_5s, NULL, &timer);
#endif
#endif

}