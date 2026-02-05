#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t blink_counter_T0 = 0;
volatile uint8_t blink_counter_T2 = 0;

#define OK 0
#define ERR 1

short init_GPIO(volatile uint8_t* ddr, uint8_t pin);

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

  init_GPIO(&DDRD, PD6); // Pin for Timer 0 (OC0A) 
  init_GPIO(&DDRB, PB1); // Pin for Timer 1 (OC1A)  
  init_GPIO(&DDRB, PB3); // Pin for Timer 2 (OC2A)

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

short init_GPIO(volatile uint8_t* ddr, uint8_t pin) {
  //*ddr allows for both B and D pins to be processed via pointer
  *ddr |= (1 << pin);
  if ((*ddr & (1 << pin)) != 0) {
    return OK;
  }else {
    return ERR;
  }
}