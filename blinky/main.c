#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define BLINK_MS 150

int main(void) {

  DDRB |= (1 << PB4);

  while (1) {
    PORTB |= (1 << PB4);
    _delay_ms(BLINK_MS);

    PORTB &= ~(1 << PB4);
    _delay_ms(BLINK_MS);
  }

  return 0;
}