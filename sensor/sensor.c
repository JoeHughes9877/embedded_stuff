#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdlib.h>

short init_GPIO(volatile uint8_t* ddr, uint8_t pin);
void lcd_trigger();
void clear_display();
void init_4_bit_lcd();
void return_home();
void entry_mode_set();
void Display_on_off_control();
void lcd_write_char(char letter);
void lcd_newline();
void lcd_write_char(char letter);
void lcd_shift_right();
void send_trigger();
void lcd_write_str(char *str);
uint16_t receive_echo();
int calculate_distance(uint16_t timeTaken);

#define OK 0
#define ERR 1
#define LCD_LONG_INSTR_DELAY 2

uint16_t distance = 0;
char buffer[8];

int main(void) {

    //---lcd

    init_GPIO(&DDRD, PD2); //Plugs into E on lcd
    init_GPIO(&DDRD, PD4); //Plugs into RS on lcd

    //BITS INTO LETTERS on lcd D3 - D7
    init_GPIO(&DDRB, PB0);
    init_GPIO(&DDRB, PB1);
    init_GPIO(&DDRB, PB2);
    init_GPIO(&DDRB, PB3);

    _delay_ms(15);
    init_4_bit_lcd();
    _delay_us(1);

    clear_display();
    return_home();
    entry_mode_set();
    Display_on_off_control();

    //---sensor

    init_GPIO(&DDRD, PD7); //Trig
    //init_GPIO(&DDRD, PD6); //Echo

    while (1) {
        send_trigger();
        distance = receive_echo();

        clear_display(); 
        return_home();   

        lcd_write_str("Distance: ");
        
        itoa(distance, buffer, 10); 
        lcd_write_str(buffer);     
        lcd_write_str(" cm");      

        _delay_ms(500); 
    }
}

//---sensor---

void send_trigger() {
    PORTD &= ~(1 << PD7); //set low to ensure stable 
    _delay_us(2);

    PORTD |= (1 << PD7); //set high to send wave thingy
    _delay_us(10); 

    PORTD &= ~(1 << PD7); 
}

uint16_t receive_echo() {
    // Wait for Echo to go high!!!
    while (!(PIND & (1 << PD6))); 

    //init the timer
    TCCR1B |= (1 << CS11);

    // Start the TIMERRRRRRRR
    TCNT1 = 0; 

    // Wait for Echo to go lowwww...
    while (PIND & (1 << PD6)); 

    //stop timer
    TCCR1B = 0;

    // SEND THE MATH OVER THE KICRUFT 
    return calculate_distance(TCNT1); 
}

int calculate_distance(uint16_t time_ticks) {
    int distance = time_ticks / 58; 
    return distance;
}

//---lcd---

void clear_display() {
    //following table 6 in HD44780 datasheet to clear the display

    PORTD &= ~(1 << PD4); //Pin RS

    //RW is plugged into gnd so no need to set to 0 

    //DB7 TO db 4 also set to 0 (0-3 can be ignored as i am in 4 bit mode)
    PORTB = PORTB & 0xF0;
    lcd_trigger();

    PORTB = (PORTB & 0xF0) | 0x01;
    lcd_trigger();
}

void return_home() {
    PORTD &= ~(1 << PD4);

    //RW is wired to gnd 

    PORTB = PORTB & 0xF0;

    lcd_trigger();

    //clean up all the pins
    PORTB = (PORTB & 0xF0) | 0x02;    

    lcd_trigger();
    _delay_ms(LCD_LONG_INSTR_DELAY);
}

void entry_mode_set() {
    PORTD &= ~(1 << PD4);

    //RW is wired to gnd 

    PORTB = PORTB & 0xF0;
    lcd_trigger();

    PORTB = (PORTB & 0xF0) | 0x6;

    lcd_trigger();
    _delay_us(40); // 37µs execution time from datasheet
}

void Display_on_off_control() {
    PORTD &= ~(1 << PD4);

    //RW is wired to gnd 

    PORTB = PORTB & 0xF0;
    lcd_trigger();
    
    PORTB = (PORTB & 0xF0) | 0xC;

    lcd_trigger();
    _delay_us(40); // Standard execution delay
}

void lcd_trigger() {
    PORTD |= (1 << PD2); 
    _delay_us(1);

    PORTD &= ~(1 << PD2); 
    _delay_ms(LCD_LONG_INSTR_DELAY);
}

void init_4_bit_lcd() {
    _delay_ms(50); //wait for power up
    PORTD &= ~(1 << PD4);

    //RW is in gnd

    for (int i = 0; i < 3; i++) {
        PORTB = (PORTB & 0xF0) | 0x3;
        lcd_trigger();
    }

    PORTB = (PORTB & 0xF0) | 0x2;
    lcd_trigger();
    _delay_us(100);
}

void lcd_write_char(char letter) {
    PORTD |= (1 << PD4);

    PORTB = (PORTB & 0xF0) | ((letter >> 4) & 0x0F);
    lcd_trigger();

    PORTB = (PORTB & 0xF0) | (letter & 0x0F);
    lcd_trigger();

    _delay_us(50); 
}

void lcd_write_str(char *str) {
    while (*str != '\0') {
        lcd_write_char(*str);
        str++;
    }
}

void lcd_shift_right() {
    PORTD &= ~(1 << PD4);

    //RW is wired to gnd 

    PORTB = (PORTB & 0xF0) | 0x1;
    lcd_trigger();

    PORTB = (PORTB & 0xF0) | 0x8;
    lcd_trigger();
}

//---general---

short init_GPIO(volatile uint8_t* ddr, uint8_t pin) {
    //*ddr allows for both B and D pins to be processed via pointer
    *ddr |= (1 << pin);
    if ((*ddr & (1 << pin)) != 0) {
        return OK;
    } else {
        return ERR;
    }
} 
