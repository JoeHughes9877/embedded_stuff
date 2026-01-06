#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define BLINK_MS 300

int main(void) {

  DDRB |= (1 << PB4); //red light
  DDRB |= (1 << PB3); //yellow light
  DDRB |= (1 << PB2); //green light

  while (1) {
    PORTB |= (1 << PB4);
    _delay_ms(BLINK_MS);

    PORTB &= ~(1 << PB4);
    _delay_ms(BLINK_MS);


    PORTB |= (1 << PB3);
    _delay_ms(BLINK_MS);

    PORTB &= ~(1 << PB3);
    _delay_ms(BLINK_MS);


    PORTB |= (1 << PB2);
    _delay_ms(BLINK_MS);

    PORTB &= ~(1 << PB2);
    _delay_ms(BLINK_MS);
  }

  return 0;
}