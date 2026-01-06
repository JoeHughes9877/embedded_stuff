#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

int main(void) {
    DDRB |= (1 << PB4); //red light
    DDRB &= ~(1 << PB2); //button
    PORTB |= (1 << PB2); 

    bool previousState = false;

  while (1) {
    bool currentState = PINB & (1 << PB2);

    if (previousState == true && currentState == false) {
      PORTB ^= (1 << PB4);
    }

    previousState = currentState;
  }
  return 0;
}