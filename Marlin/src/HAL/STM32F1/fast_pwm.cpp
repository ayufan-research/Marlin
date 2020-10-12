#ifdef __STM32F1__

#include "../../inc/MarlinConfigPre.h"
#include "../../MarlinCore.h"

#if NEEDS_HARDWARE_PWM // Specific meta-flag for features that mandate PWM

#include "HAL.h"

#define MAX_RELOAD ((1 << 16) - 1)

void set_pwm_frequency(const pin_t pin, int f_desired) {
  if (pin >= BOARD_NR_GPIO_PINS) {
      return;
  }

  // Recv: echo:set_pwm_frequency: pin:38 f_desired:141176 period_cyc:504 prescaler:1 overflow:504
  // Recv: echo:set_pwm_frequency: pin:39 f_desired:141176 period_cyc:504 prescaler:1 overflow:504

  timer_dev *dev = PIN_MAP[pin].timer_device;
  if (!dev) {
    return;
  }

  uint32 microseconds = 1000000 / f_desired;
  uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND;
  uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
  uint16 overflow = (uint16)((period_cyc + (prescaler / 2)) / prescaler);

  timer_set_prescaler(dev, (uint16)(prescaler - 1));
  timer_set_reload(dev, overflow);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("set_pwm_frequency:");
  SERIAL_ECHOPAIR_F(" pin:", pin);
  SERIAL_ECHOPAIR_F(" f_desired:", f_desired);
  SERIAL_ECHOPAIR_F(" period_cyc:", period_cyc);
  SERIAL_ECHOPAIR_F(" prescaler:", prescaler);
  SERIAL_ECHOPAIR_F(" overflow:", overflow);
  SERIAL_EOL();
}

void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size/*=255*/, const bool invert/*=false*/) {
  if (pin >= BOARD_NR_GPIO_PINS) {
      return;
  }

  timer_dev *dev = PIN_MAP[pin].timer_device;
  if (!dev) {
    return;
  }

  uint8 cc_channel = PIN_MAP[pin].timer_channel;
  uint16 reload = timer_get_reload(dev);
  uint16 duty_cycle = (invert ? 1.0f - (float)v / v_size : (float)v / v_size) * reload;
  timer_set_compare(dev, cc_channel, duty_cycle);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("set_pwm_frequency:");
  SERIAL_ECHOPAIR_F(" pin:", pin);
  SERIAL_ECHOPAIR_F(" v:", v);
  SERIAL_ECHOPAIR_F(" v_size:", v_size);
  SERIAL_ECHOPAIR_F(" invert:", invert);
  SERIAL_ECHOPAIR_F(" reload:", reload);
  SERIAL_ECHOPAIR_F(" duty_cycle:", duty_cycle);
  SERIAL_EOL();

  // Recv: echo:set_pwm_frequency: pin:38 v:0 v_size:255 invert:0
  // Recv: echo:set_pwm_frequency: pin:38 v:0 v_size:255 invert:0
}

#endif // NEEDS_HARDWARE_PWM
#endif // __STM32F1__
