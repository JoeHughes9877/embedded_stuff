#include <avr/io.h>
#include <stdbool.h>

#define OK 0
#define ERR 1

short init_GPIO(volatile uint8_t* ddr, uint8_t pin);

int main(void) {
    DDRB |= (1 << PB4); //red light
    PORTB |= (1 << PB2); 

    init_GPIO(&DDRB, PB4);
    init_GPIO(&DDRB, PB2);
    DDRB &= ~(1 << PB2); //button

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

short init_GPIO(volatile uint8_t* ddr, uint8_t pin) {
  //*ddr allows for both B and D pins to be processed via pointer
  *ddr |= (1 << pin);
  if ((*ddr & (1 << pin)) != 0) {
    return OK;
  }else {
    return ERR;
  }
}