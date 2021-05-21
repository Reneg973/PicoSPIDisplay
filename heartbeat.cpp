#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/time.h"

#define LED_DURATION_ON_MS  1
#define INTERVAL_DURATION_MS  5000

namespace
{
  int64_t heartbeatTimerAlarm(alarm_id_t id, void *user_data)
  {
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    return 0;
  }

  bool heartbeat_interval(struct repeating_timer *t)
  {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    add_alarm_in_ms(LED_DURATION_ON_MS, &heartbeatTimerAlarm, NULL, false);
    return true;
  }

  struct repeating_timer timer;
}

void initHeartBeat()
{
#ifndef PICO_DEFAULT_LED_PIN
#warning PICO_DEFAULT_LED_PIN not defined
#else
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  add_repeating_timer_ms(INTERVAL_DURATION_MS, heartbeat_interval, NULL, &timer);
#endif
}