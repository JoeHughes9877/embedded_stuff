#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t blink_counter_T0 = 0;
volatile uint8_t blink_counter_T2 = 0;

ISR (TIMER0_OVF_vect) {
  blink_counter_T0++;

  if (blink_counter_T0 == 200) {
    PORTD ^= _BV(PORTD6);
    blink_counter_T0 = 0;
  }
}

ISR (TIMER2_OVF_vect) {
  blink_counter_T2++;

  if (blink_counter_T2 == 255) {
    PORTB ^= _BV(PORTB3);
    blink_counter_T2 = 0;
  }
}

int main(void) {

  DDRD |= (1 << PD6);    // Pin for Timer 0 (OC0A)
  DDRB |= (1 << PB1);    // Pin for Timer 1 (OC1A)
  DDRB |= (1 << PB3);    // Pin for Timer 2 (OC2A)

  //making timer control register same as the compare output mode
  TCCR1A |= (1 << COM1A0); 

    //timer 0
    TCCR0B |= (1 << CS00) | (1 << CS02);

   //allows cpu to be interuppted by overflow
    TIMSK0 = (1 << TOIE0);

    sei();

    //timer 1
    OCR1A = 2000;
    TCCR1B |= (1 << CS10) | (1 << CS12);

    //timer 2
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

    //allows cpu to be interuppted by overflow
    TIMSK2 = (1 << TOIE2);


  while (1) {
    continue;
    }

  return 0;
}