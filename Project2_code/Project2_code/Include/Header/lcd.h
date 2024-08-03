/*
 * lcd.h
 *
 * Created: 02/08/2024 10:55:19
 * Author: Test
 */ 

#ifndef LCD_H_
#define LCD_H_

// Define CPU frequency as 8MHz
#define F_CPU 8000000UL 

// Include AVR input/output header
#include <avr/io.h>
// Include utility for delays
#include <util/delay.h>
// Include standard input/output functions
#include <stdio.h>
// Include string manipulation functions
#include <string.h>

// Define LCD control pins
#define LCD_RS PD6
#define LCD_RW PD5
#define LCD_EN PD7
// Define the degree symbol for LCD display
#define degree_symbol 0xdf
// Define LCD data pins
#define LCD_D4 PC4
#define LCD_D5 PC5
#define LCD_D6 PC6
#define LCD_D7 PC7

// Define ADC voltage reference type
#define ADC_VREF_TYPE 0x00

// Buffer for storing strings
char buff[5];
// Variable to store ADC value
int adc_value;
// Variable to store temperature value
float nhietdo;

// Function to initialize the ADC
void ADC_Init(){
    // Set ADC port as input
    DDRA = 0x00; 
    // Enable ADC with frequency/128
    ADCSRA = 0x87; 
    // Set Vref: Avcc and ADC channel: 0
    ADMUX = 0x40; 
}

// Function to read value from ADC channel
int ADC_Read(char channel)
{
    // Set input channel to read
    ADMUX = 0x40 | (channel & 0x07); 
    // Start ADC conversion
    ADCSRA |= (1<<ADSC); 
    // Wait until end of conversion by polling ADC interrupt flag
    while (!(ADCSRA & (1<<ADIF))); 
    // Clear interrupt flag by setting the ADIF bit of ADCSRA register
    ADCSRA |= (1<<ADIF); 
    // Wait a little bit
    _delay_ms(1); 
    // Return ADC word
    return ADCW; 
}

// Function to create an enable pulse on EN pin for LCD to accept data
void LCD_Enable(void)
{
    // Set EN pin
    PORTD |= (0x01 << LCD_EN);
    // Small delay
    _delay_us(3);
    // Clear EN pin
    PORTD &= ~(0x01 << LCD_EN);
    // Small delay
    _delay_us(50);
}

// Function to send 4 bits of data to the LCD
void LCD_Send4Bit(unsigned char Data)
{
    // Send bit 0
    (Data & 1) ? (PORTC |= (1 << LCD_D4)) : (PORTC &= ~(0x01 << LCD_D4));
    // Send bit 1
    ((Data >> 1) & 1) ? (PORTC |= (1 << LCD_D5)) : (PORTC &= ~(0x01 << LCD_D5));
    // Send bit 2
    ((Data >> 2) & 1) ? (PORTC |= (1 << LCD_D6)) : (PORTC &= ~(0x01 << LCD_D6));
    // Send bit 3
    ((Data >> 3) & 1) ? (PORTC |= (1 << LCD_D7)) : (PORTC &= ~(0x01 << LCD_D7));
}

// Function to send a command to the LCD
void LCD_SendCommand(unsigned char command)
{
    // Send higher nibble first
    LCD_Send4Bit((unsigned char)command >> 4);
    // Enable pulse
    LCD_Enable();
    // Send lower nibble
    LCD_Send4Bit(command);
    // Enable pulse
    LCD_Enable();
}

// Function to clear the LCD screen
void LCD_Clear()
{
    // Clear display command
    LCD_SendCommand(0x01);
    // Delay for command execution
    _delay_ms(2);
}

// Function to enable the blinking cursor on the LCD
void LCD_BlinkPointer()
{
    // Display on, cursor on, blink off
    LCD_SendCommand(0x0E);
}

// Function to enable the blinking cursor plus on the LCD
void LCD_BlinkPointerPlus()
{
    // Display on, cursor on, blink on
    LCD_SendCommand(0x0F);
}

// Function to initialize the LCD
void LCD_Init()
{
    // Send initial dummy data
    LCD_Send4Bit(0x00); // Turn on LCD
    _delay_ms(20); // Wait for LCD to power up
    // Set RS and RW to 0 for command mode
    PORTD &= ~(0x01 << LCD_RS);
    PORTD &= ~(0x01 << LCD_RW);
    // Function setting: 8-bit mode
    LCD_Send4Bit(0x03);
    LCD_Enable();
    _delay_ms(5); // Wait for command to execute
    LCD_Enable();
    _delay_us(100); // Short delay
    LCD_Enable();
    // Set LCD to 4-bit mode
    LCD_Send4Bit(0x02);
    LCD_Enable();
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    LCD_SendCommand(0x28);
    // Display on, cursor off, blink off
    LCD_SendCommand(0x0c);
    // Entry mode set: increment cursor, no display shift
    LCD_SendCommand(0x06);
    // Clear the LCD screen
    LCD_Clear();
}

// Function to set the cursor position on the LCD
void LCD_GotoXY(unsigned char x, unsigned char y)
{
    unsigned char address;
    // If y is 0, go to the first line
    if(!y) address = (0x80 + x);
    // Otherwise, go to the second line
    else address = (0xc0 + x);
    // Set the address in the LCD
    LCD_SendCommand(address);
}

// Function to write a character to the LCD
void LCD_PutChar(unsigned char Data)
{
    // Set RS to 1 for data mode
    PORTD |= (0x01 << LCD_RS);
    // Send data
    LCD_SendCommand(Data);
    // Set RS back to 0
    PORTD &= ~(0x01 << LCD_RS);
}

// Function to write a string to the LCD
void LCD_Puts(char *s)
{
    // Loop through each character in the string
    while (*s)
    {
        // Write the character to the LCD
        LCD_PutChar(*s);
        // Move to the next character
        s++;
    }
}

#endif /* LCD_H_ */
