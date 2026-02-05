#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define BLINK_MS 300

#define OK 0
#define ERR 1

short init_GPIO(volatile uint8_t* ddr, uint8_t pin);

int main(void) {

  const uint8_t leds[] = { PD7, PB3, PB2 };
  volatile uint8_t* ports[] = {&PORTD, &PORTB, &PORTB};


  uint8_t amountOfLeds = sizeof(leds) / sizeof(leds[0]);

  init_GPIO(&DDRD, PD7);
  init_GPIO(&DDRB, PB3);
  init_GPIO(&DDRB, PB2);

  while (1) {
    for (uint8_t i = 0; i < amountOfLeds; i++) {
      *ports[i] |= (1 << leds[i]);
      _delay_ms(BLINK_MS);

      *ports[i] &= ~(1 << leds[i]);
      _delay_ms(BLINK_MS);
    }
  }

  return 0;
}

short init_GPIO(volatile uint8_t* ddr, uint8_t pin) {
  //*ddr allows for both B and D pins to be processed via pointer
  *ddr |= (1 << pin);
  if ((*ddr & (1 << pin)) != 0) {
    return OK;
  }else {
    return ERR;
  }
}