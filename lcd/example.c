// Ai generated example of LCD display code in c that works with my current setup that 
// i can refrence and use as an example when creating my own implementation

#include <avr/io.h>
#include <util/delay.h>

// Data remains on PD2, PD3, PD4, PD5 (Digital 2, 3, 4, 5)
#define LCD_PORT PORTD
#define LCD_DDR  DDRD

// Control Pins Swapped to match your description:
#define EN PB3  // Digital 11 -> E
#define RS PB4  // Digital 12 -> RS

void lcd_latch() {
    PORTB |= (1 << EN);   // Set Enable High
    _delay_us(1);
    PORTB &= ~(1 << EN);  // Set Enable Low
    _delay_us(100);
}

void lcd_command(unsigned char cmd) {
    PORTB &= ~(1 << RS);  // RS = 0 for command
    LCD_PORT = (LCD_PORT & 0x0F) | (cmd & 0xF0); 
    lcd_latch();
    LCD_PORT = (LCD_PORT & 0x0F) | (cmd << 4);
    lcd_latch();
    _delay_ms(2);
}

void lcd_char(unsigned char data) {
    PORTB |= (1 << RS);   // RS = 1 for data
    LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0);
    lcd_latch();
    LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);
    lcd_latch();
}

void lcd_init() {
    LCD_DDR |= 0xFC; // Pins 2-5 as output
    DDRB |= (1<<RS) | (1<<EN) | (1<<PB5); // Pins 11, 12, 13 as output
    
    _delay_ms(100);
    
    // 4-bit Initialization Sequence
    LCD_PORT = (LCD_PORT & 0x0F) | 0x30; lcd_latch(); _delay_ms(5);
    LCD_PORT = (LCD_PORT & 0x0F) | 0x30; lcd_latch(); _delay_us(200);
    LCD_PORT = (LCD_PORT & 0x0F) | 0x30; lcd_latch();
    LCD_PORT = (LCD_PORT & 0x0F) | 0x20; lcd_latch(); 

    lcd_command(0x28); // 2 lines, 5x7 matrix
    lcd_command(0x0C); // Display ON, Cursor OFF
    lcd_command(0x01); // Clear Screen
    _delay_ms(5);
}

int main(void) {
    lcd_init();
    while (1) {
        PINB |= (1 << PB5); // Blink LED L (Pin 13)
        
        lcd_command(0x80);  // Move to line 1
        for(char *p = "Hello!"; *p; p++) lcd_char(*p);
        
        _delay_ms(500);
    }
}