#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define BLINK_MS 300

int main(void) {

  const uint8_t leds[] = { PB4, PB3, PB2 };

  uint8_t amountOfLeds = sizeof(leds) / sizeof(leds[0]);

  DDRB |= (1 << PB4); //red light
  DDRB |= (1 << PB3); //yellow light
  DDRB |= (1 << PB2); //green light

  while (1) {
    for (uint8_t i = 0; i < amountOfLeds; i++) {
      PORTB |= (1 << leds[i]);
      _delay_ms(BLINK_MS);

      PORTB &= ~(1 << leds[i]);
      _delay_ms(BLINK_MS);
    }
  }

  return 0;
}