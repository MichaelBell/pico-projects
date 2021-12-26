#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define BASE_PIN 6
#define NUM_PINS 8

int main()
{
  gpio_init(25);
  gpio_set_dir(25, 1);
  gpio_put(25, 1);

  stdio_init_all();
  sleep_ms(2000);
  printf("Hello, world\n");

  adc_init();

  // Speed control pot
  adc_gpio_init(28);
  adc_select_input(2);
  uint16_t speed = 2048 + (adc_read() >> 1);
  adc_select_input(1);

  // Light sensor - we're powering from GPIO pin 26 which
  // is a bit naughty but it draws hardly anything so fine.
  adc_gpio_init(27);
  gpio_init(26);
  gpio_set_dir(26, 1);
  gpio_put(26, 1);
  uint16_t brightness = adc_read();

  printf("Speed: %hu\t Bright: %hu\n", speed, brightness);

  for (int i = 0; i < 5; ++i) {
    sleep_ms(200);
    gpio_put(25, 0);
    sleep_ms(200);
    gpio_put(25, 1);
  }

#if 1
#if 1
  for (int pin = BASE_PIN; pin < BASE_PIN+NUM_PINS; ++pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_gpio_level(pin, 0);
  }
  for (int pin = BASE_PIN; pin < BASE_PIN+NUM_PINS; pin += 2) {
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 4095);
    pwm_set_clkdiv(slice, 32768.f);
    pwm_set_enabled(slice, true);
  }

  for (int pin = BASE_PIN; pin < BASE_PIN+NUM_PINS; pin += 2) {
    pwm_set_gpio_level(pin, speed);
  }
  sleep_ms(1000);
  for (int pin = BASE_PIN; pin < BASE_PIN+NUM_PINS; pin += 2) {
    pwm_set_gpio_level(pin, 0);
  }
  
#else
  for (int pin = BASE_PIN; pin < BASE_PIN+NUM_PINS; ++pin) {
    gpio_init(pin);
    gpio_set_dir(pin, 1);
    gpio_put(pin, 0);
  }

  gpio_put_masked(0b11111111 << BASE_PIN, 0b01010101 << BASE_PIN);
  sleep_ms(2000);
  gpio_put_masked(0b11111111 << BASE_PIN, 0b00000000 << BASE_PIN);
#endif

  gpio_put(25, 0);
#endif
  return 0;
}
