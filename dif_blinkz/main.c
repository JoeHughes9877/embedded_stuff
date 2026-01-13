#include <avr/io.h>
#include <stdbool.h>

int main(void) {

  DDRD |= (1 << PD6);    // Pin for Timer 0 (OC0A)
  DDRB |= (1 << PB1);    // Pin for Timer 1 (OC1A)
  DDRB |= (1 << PB3);    // Pin for Timer 2 (OC2A)
    
  //making timer control register same as the compare output mode
  TCCR0A |= (1 << COM0A0); 
  TCCR1A |= (1 << COM1A0); 
  TCCR2A |= (1 << COM2A0); 

    //timer 0
    OCR0A = 220;
    TCCR0B |= (1 << CS02) | (1 << CS00);

    //timer 1
    OCR1A = 15625;
    TCCR1B |= (1 << CS12) | (1 << CS10);

    //timer 2
    OCR2A = 170;
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);


  while (1) {
    continue;
    }

  return 0;
}